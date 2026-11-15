"""Room track from the delivered CAD primitives, with screened gate candidates."""
from pathlib import Path
import json, math
import numpy as np
from scipy.spatial import cKDTree
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import unary_union
from shapely import contains_xy
from course import Course

class RoomCourse(Course):
    def __init__(self,gate_mode='all',gate_width=.6,gap_length=.6):
        self.kind='room';self.gate_x=6.1;self.gate_width=gate_width
        data=json.loads((Path(__file__).parent/'room_track.json').read_text(encoding='utf-8'))
        pts=[];tags=[];self.elements=[];self.primitives=data['primitives']
        for p in self.primitives:
            if p['kind']=='line':
                a=np.array(p['a']);b=np.array(p['b']);ps=np.linspace(a,b,max(2,math.ceil(np.linalg.norm(b-a)/.012)+1))
            else:
                t=np.radians(np.linspace(p['start_deg'],p['end_deg'],max(3,math.ceil(abs(p['end_deg']-p['start_deg'])*math.pi/180*p['radius']/.012)+1)))
                ps=np.array(p['centre'])+p['radius']*np.c_[np.cos(t),np.sin(t)]
            first=max(0,len(pts)-1)
            for xy in ps:
                if pts and np.linalg.norm(pts[-1]-xy)<1e-8:continue
                pts.append(xy);tags.append(p['element'])
            self.elements.append(dict(name=p['element'],first=first,last=len(pts)-1,id=p['id']))
        self.points=np.array(pts);self.tags=tags
        self.ds=np.linalg.norm(np.diff(self.points,axis=0),axis=1);self.s=np.r_[0,np.cumsum(self.ds)];self.length=float(self.s[-1])
        self.tree=cKDTree(self.points);self._road_trees={};self._gap_shapes={};self._road_shapes={}
        self.room=data['room']['vertices'];self.room_shape=Polygon(self.room)
        self.source=dict(title='房间黑白定制赛道',labelled_length_m=data['centreline_length'],status='来自已交付直道/圆弧几何，门位为训练扩展')
        self.start=[2,.4,0];self.targets=[]
        self.features=[dict(kind='ramp',name='可拆坡道',centre=data['ramp']['centre'],heading=math.pi,length=1.2,width=.45,height=.025),
                       dict(kind='obstacle',name='可拆路障',centre=data['obstacle']['centre'],heading=0,length=.25,width=.12,height=.035)]
        self.markers=[dict(name=n,x=xy[0],y=xy[1]) for n,xy in [('左圆环',data['circle_centres'][0]),('右圆环',data['circle_centres'][1]),
             ('Ω弯',[5.966,4.1]),('正十字',data['crosses'][0]),('斜十字',data['crosses'][1]),('S弯',[5.107,.55]),('起终点',[2.4,.4])]]
        # Search each element, keeping gate feet inside the room and off other roads.
        requests=[('straight','直道',[8],[.45,.65,.25]),('left_ring','左圆环',[2],[.5,.42,.58,.35,.65]),
          ('s_bend','S弯',[5,6,4,7],[.6,.4,.8,.2]),('hairpin','回头弯',[9],[.5,.4,.6]),
          ('original','原直道',[10],[.375]),('cross','正十字出口段',[13,12],[.65,.5,.8,.35]),
          ('omega','Ω弯',[16],[.5,.4,.6]),('ramp','坡道出口段',[23],[.78,.9]),
          ('obstacle','路障接近段',[22,23],[.45,.25,.08,.12]),('diagonal','斜十字接近段',[25,27,24,28],[.1,.9,.25,.75]),
          ('right_ring','右圆环',[26],[.5,.4,.6])]
        self.gate_catalog=[];chosen=[]
        for ident,label,ids,fractions in requests:
            candidates=[]
            for pid in ids:
                e=next(e for e in self.elements if e['id']==pid)
                for f in fractions:
                    distance=self.s[e['first']]+f*(self.s[e['last']]-self.s[e['first']]);centre,heading=self.pose(distance)
                    normal=np.array([-math.sin(heading),math.cos(heading)])
                    feet=[Point(centre+sign*normal*gate_width/2).buffer(.025) for sign in [-1,1]]
                    mask=np.minimum(abs(self.s-distance),self.length-abs(self.s-distance))<.55
                    # Distances to nonlocal route samples, avoiding synthetic line joins.
                    nonlocal_points=self.points[~mask]
                    nearest=cKDTree(nonlocal_points).query(np.array([np.array(q.centroid.coords[0]) for q in feet]))[0]-.225-.025
                    own=cKDTree(self.points).query(np.array([np.array(q.centroid.coords[0]) for q in feet]))[0]-.225-.025
                    room_clear=min(self.room_shape.boundary.distance(q) if self.room_shape.contains(q) else -1 for q in feet)
                    clearance=min(float(nearest.min()),float(own.min()),room_clear)
                    if any(np.linalg.norm(centre-np.array(g['centre']))<.8 for g in chosen):clearance=-1
                    candidates.append((clearance,distance,centre,heading))
            best=max(candidates,key=lambda x:x[0]);clearance,distance,centre,heading=best
            entry=dict(id=ident,name=label,centre=centre.tolist(),heading=heading,progress=distance,
                       available=clearance>=.012,foot_clearance_mm=round(clearance*1000,1),
                       reason='' if clearance>=.012 else '门脚与邻道或其他门位过近，保留为单门可选场景')
            self.gate_catalog.append(entry)
            if entry['available']:chosen.append(entry)
        if gate_mode=='all':self.gates=chosen
        elif gate_mode=='none':self.gates=[]
        else:
            selected=next((g for g in self.gate_catalog if g['id']==gate_mode),None)
            if selected is None:raise ValueError('无效门位：'+gate_mode)
            self.gates=[selected]
        self.gate=(self.gates or [self.gate_catalog[4]])[0]
        self.tangent=np.array([math.cos(self.gate['heading']),math.sin(self.gate['heading'])]);self.normal=np.array([-self.tangent[1],self.tangent[0]])
        self.configure_gaps(gap_length)

    def pose(self,progress):
        value=float(progress)%self.length
        i=min(len(self.ds)-1,max(0,np.searchsorted(self.s,value,side='right')-1))
        v=self.points[i+1]-self.points[i];t=(value-self.s[i])/max(self.ds[i],1e-10)
        return self.points[i]+t*v,math.atan2(v[1],v[0])

    def segment_points(self,centre,length):
        return np.array([self.pose(s)[0] for s in np.linspace(centre-length/2,centre+length/2,max(12,math.ceil(length/.012)+1))])

    def configure_gaps(self,gap):
        self.gap_length=gap
        self._road_shapes.clear()
        self.gap_regions=unary_union([LineString(self.segment_points(g['progress'],gap)).buffer(.226,cap_style='flat',quad_segs=24) for g in self.gates])
        full=LineString(self.points).buffer(.225,quad_segs=24,cap_style='flat')
        self.visible_shape=full.difference(self.gap_regions)
        self._road_shapes[.45]=self.visible_shape

    def gap_mask(self,points,gap):
        if gap!=self.gap_length:self.configure_gaps(gap)
        p=np.asarray(points)
        return contains_xy(self.gap_regions,p[...,0],p[...,1])

    def is_road(self,xy,width,gap):
        if gap!=self.gap_length:self.configure_gaps(gap)
        if width not in self._road_shapes:self._road_shapes[width]=LineString(self.points).buffer(width/2,cap_style='flat').difference(self.gap_regions)
        p=np.asarray(xy);visible=contains_xy(self._road_shapes[width],p[...,0],p[...,1])
        for f in self.features:
            if f['kind']=='obstacle':
                d=p-np.array(f['centre']);visible &= ~((abs(d[...,0])<f['length']/2)&(abs(d[...,1])<f['width']/2))
        return visible

    def element(self,progress,index):
        xy=self.points[index]
        if np.linalg.norm(xy-np.array([4.6,2.05]))<.27:return '正十字',2
        if np.linalg.norm(xy-np.array([2.9757359,2.2671573]))<.25:return '斜十字',2
        name=self.tags[index]
        if '圆环' in name:return name,4 if '左' in name else 7
        if name=='坡道/路障直道':
            if 4.9<xy[0]<6.1:return '坡道',1
            if xy[0]>6.5:return '路障接近段',1
        return name,1 if name!='直道' and '直道' not in name else 0

    def payload(self):
        result=super().payload();result.update(gates=self.gates,gate_catalog=self.gate_catalog,room=self.room,
          point_progress=self.s.tolist(),gap_mask=self.gap_mask(self.points,self.gap_length).tolist(),
          boundary_rings=[list(p.exterior.coords) for p in ([self.visible_shape] if self.visible_shape.geom_type=='Polygon' else self.visible_shape.geoms)]+
                         [list(r.coords) for p in ([self.visible_shape] if self.visible_shape.geom_type=='Polygon' else self.visible_shape.geoms) for r in p.interiors])
        return result
