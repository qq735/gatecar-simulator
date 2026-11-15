"""Editable synthetic closed course. Dimensions are design assumptions, not rules."""
import math
import numpy as np
from scipy.spatial import cKDTree

class Course:
    def __init__(self,kind='closed',gate_x=3.8):
        self.kind=kind;self.gate_x=gate_x;points=[];tags=[];self.elements=[]
        def append(ps,tag):
            start=len(points)
            for p in ps:
                p=np.asarray(p,dtype=float)
                if points and np.linalg.norm(p-points[-1])<1e-6:continue
                points.append(p);tags.append(tag)
            self.elements.append(dict(name=tag,first=start,last=len(points)-1))
        def line(a,b,tag='直道'):
            n=max(2,int(np.linalg.norm(np.array(b)-a)/.035)+1);append(np.linspace(a,b,n),tag)
        def arc(c,r,start,end,tag='弯道'):
            n=max(3,int(abs(end-start)*math.pi/180*r/.035)+1)
            angles=np.radians(np.linspace(start,end,n));append(np.c_[c[0]+r*np.cos(angles),c[1]+r*np.sin(angles)],tag)
        self.features=[];self.targets=[];self.source=None
        if kind=='reference_2026':
            from reference_course import make_reference
            ref=make_reference();points=list(ref['points']);tags=ref['tags'];self.elements=ref['elements'];self.markers=ref['markers'];self.features=ref['features'];self.targets=ref['targets'];self.source=ref['source'];self.start=ref['start'];self.gate=ref['gate']
        elif kind=='gate_only':
            line([-.5,0],[8,0]);self.markers=[dict(name='电磁门',x=gate_x,y=0)]
        else:
            line([1,0],[6,0]);arc([6,.9],.9,-90,270,'左圆环')
            line([6,0],[8,0]);arc([8,1],1,-90,0)
            line([9,1],[9,6]);arc([10,6],1,180,90)
            # Catmull-Rom path makes the Omega detour visibly distinct from a simple U bend.
            control=np.array([[10,7],[10.5,7],[11.1,7.25],[10.9,7.95],[10.9,8.65],[11.5,9.15],
                [12.4,9.35],[13.3,9.15],[13.9,8.65],[13.9,7.95],[13.7,7.25],[14.2,7],[14.6,7]])
            ext=np.vstack([control[0]-(control[1]-control[0]),control,control[-1]+control[-1]-control[-2]])
            ps=[]
            for i in range(1,len(ext)-2):
                a,b,c,d=ext[i-1:i+3]
                for t in np.linspace(0,1,25,endpoint=False):
                    ps.append(.5*((2*b)+(-a+c)*t+(2*a-5*b+4*c-d)*t*t+(-a+3*b-3*c+d)*t*t*t))
            ps.append(control[-1]);append(ps,'Ω形')
            line([14.6,7],[15,7]);arc([15,6],1,90,0)
            line([16,6],[16,4]);arc([15,4],1,0,-90)
            line([15,3],[5.5,3]);arc([5.5,3.9],.9,-90,-450,'右圆环')
            line([5.5,3],[1,3]);arc([1,2],1,90,180)
            line([0,2],[0,1]);arc([1,1],1,180,270)
            self.markers=[dict(name='左圆环',x=6,y=.9),dict(name='十字',x=9,y=3),dict(name='Ω形',x=12.4,y=8.8),dict(name='右圆环',x=5.5,y=3.9),dict(name='电磁门',x=gate_x,y=0),dict(name='起终点',x=1.2,y=0)]
        if kind!='reference_2026':self.gate=dict(centre=[gate_x,0],heading=0);self.start=None
        self.points=np.array(points);self.tags=tags
        self.ds=np.linalg.norm(np.diff(self.points,axis=0),axis=1);self.s=np.r_[0,np.cumsum(self.ds)]
        self.length=float(self.s[-1]);self.tree=cKDTree(self.points)
        self._road_trees={}
        self.tangent=np.array([math.cos(self.gate['heading']),math.sin(self.gate['heading'])]);self.normal=np.array([-self.tangent[1],self.tangent[0]])
    def gap_mask(self,points,gap):
        delta=np.asarray(points)-np.asarray(self.gate['centre']);return (np.abs(delta@self.tangent)<gap/2)&(np.abs(delta@self.normal)<.26)
    def height(self,xy):
        p=np.asarray(xy);z=np.zeros(p.shape[:-1])
        for f in self.features:
            if f['kind']!='ramp':continue
            d=p-np.array(f['centre']);u=d@np.array([math.cos(f['heading']),math.sin(f['heading'])]);v=d@np.array([-math.sin(f['heading']),math.cos(f['heading'])]);inside=(abs(u)<f['length']/2)&(abs(v)<f['width']/2+.01)
            z=np.where(inside,f['height']*(1+np.cos(np.pi*u/(f['length']/2)))/2,z)
        return z
    def is_road(self,xy,width,gap):
        p=np.asarray(xy);flat=p.reshape(-1,2)
        keep=~self.gap_mask(self.points,gap)
        if gap not in self._road_trees:self._road_trees[gap]=cKDTree(self.points[keep])
        tree=self._road_trees[gap];good=np.isfinite(flat).all(axis=1);d=np.full(len(flat),1e6)
        d[good]=tree.query(flat[good])[0]
        visible=(d<width/2).reshape(p.shape[:-1])
        # Keep the requested interruption explicit even near the rounded ends of the road mask.
        visible &= ~self.gap_mask(p,gap)
        for f in self.features:
            if f['kind']=='obstacle':
                delta=p-np.array(f['centre']);visible&=~((abs(delta[...,0])<f['length']/2)&(abs(delta[...,1])<f['width']/2))
        return visible
    def locate(self,xy,progress=None):
        p=np.asarray(xy)
        if self.kind=='gate_only':
            dist,index=self.tree.query(p);return float(self.s[index]),float(dist),int(index)
        if progress is None:
            dist,index=self.tree.query(p);return float(self.s[index]),float(dist),int(index)
        lap=math.floor(progress/self.length);base=progress%self.length
        indices=np.where((self.s>=base-.45)&(self.s<=base+1.0))[0];laps=np.full(len(indices),lap)
        if base<.45:
            more=np.where(self.s>=self.length+base-.45)[0];indices=np.r_[indices,more];laps=np.r_[laps,np.full(len(more),lap-1)]
        if base>self.length-1:
            more=np.where(self.s<=base+1-self.length)[0];indices=np.r_[indices,more];laps=np.r_[laps,np.full(len(more),lap+1)]
        j=int(np.argmin(np.linalg.norm(self.points[indices]-p,axis=1)));index=int(indices[j]);dist=float(np.linalg.norm(self.points[index]-p))
        return float(self.s[index]+laps[j]*self.length),dist,index
    def target(self,progress,lookahead=.4):
        desired=progress+lookahead
        if self.kind!='gate_only':desired%=self.length
        return self.points[min(len(self.s)-1,int(np.searchsorted(self.s,max(0,desired))))]
    def element(self,progress,index):
        p=self.points[index]
        if self.kind=='closed' and abs(p[0]-9)<.55 and abs(p[1]-3)<.55:return '十字',2
        if self.kind=='reference_2026':
            from reference_course import world
            if np.linalg.norm(p-world([1176,717]))<.35:return '斜十字',2
            if min(np.linalg.norm(p-world([596,795])),np.linalg.norm(p-world([404,795])))<.30:return '十字',2
            if np.linalg.norm(p-world([404,490]))<.6:return '坡道',1
        name=self.tags[index]
        mapping={'直道':0,'弯道':1,'Ω形':1,'左圆环':4,'右圆环':7}
        code=mapping.get(name,0)
        if name in ['左圆环','右圆环']:
            element=next(x for x in self.elements if x['name']==name);local=self.s[index]-self.s[element['first']];remaining=self.s[element['last']]-self.s[index]
            code=(3 if local<.4 else 5 if remaining<.4 else 4) if name=='左圆环' else (6 if local<.4 else 8 if remaining<.4 else 7)
        return name,code
    def payload(self):
        bounds=dict(min=self.points.min(axis=0).tolist(),max=self.points.max(axis=0).tolist())
        return dict(kind=self.kind,points=self.points.tolist(),length_m=self.length,markers=self.markers,features=self.features,targets=self.targets,source=self.source,gate=self.gate,bounds=bounds,
            element_tags=self.tags,dimensions_status='User drawing reconstruction plus added gate' if self.kind=='reference_2026' else 'Synthetic test course; not an official competition layout')
