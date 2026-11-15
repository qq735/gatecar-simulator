"""Reconstruction of the user-supplied 2026 course drawing, plus an added gate."""
import math
import numpy as np

SCALE=192.0
def world(p):return np.array([(p[0]-116)/SCALE,(1169-p[1])/SCALE],dtype=float)

def make_reference():
    pixels=[];tags=[];elements=[]
    def append(ps,tag):
        start=len(pixels)
        for p in ps:
            if pixels and np.linalg.norm(np.array(p)-pixels[-1])<1e-6:continue
            pixels.append(np.asarray(p,dtype=float));tags.append(tag)
        elements.append(dict(name=tag,first=start,last=len(pixels)-1))
    def line(end,tag='直道'):
        a=pixels[-1] if pixels else np.array(end);b=np.array(end)
        n=max(2,int(np.linalg.norm(b-a)/3.5)+1);append(np.linspace(a,b,n),tag)
    def arc(c,r,start,end,tag='弯道'):
        n=max(4,int(abs(end-start)*math.pi/180*r/3.5)+1);a=np.radians(np.linspace(start,end,n));append(np.c_[c[0]+r*np.cos(a),c[1]+r*np.sin(a)],tag)
    r=96.;xr=1303.5537550532245;xc=xr-r;ringx=1041.2768775266122
    append([[xr,453]],'起终点')
    line([xr,549]);arc([xc,549],r,0,45)
    line([ringx+r/math.sqrt(2),717+r/math.sqrt(2)],'斜十字')
    arc([ringx,717],r,45,315,'右圆环')
    line([xc+r/math.sqrt(2),883-r/math.sqrt(2)],'斜十字')
    arc([xc,883],r,315,360)
    line([xr,954]);arc([xc,954],r,0,90)
    line([928,1050],'路障直道');arc([928,954],r,90,180)
    line([832,891]);arc([736,891],r,0,-90)
    line([255,795],'十字直道');arc([255,699],r,90,180)
    line([159,261],'门区直道');arc([255,261],r,180,270)
    line([308,165]);arc([308,261],r,270,360)
    line([404,1029],'坡道直道');arc([500,1029],r,180,0)
    line([596,506],'十字直道');arc([692,506],r,180,540,'左圆环')
    line([596,261]);arc([692,261],r,180,270)
    line([875,165]);arc([875,261],r,270,360)
    arc([875,261],r,0,30,'Ω形')
    arc([ringx,357],r,210,-30,'Ω形')
    arc([xc,261],r,150,180,'Ω形')
    arc([xc,261],r,180,360);line([xr,453])
    pts=np.array([world(p) for p in pixels])
    marker_specs=[('起终点',[xr+67,410]),('Ω形',[1041,357]),('左圆环',[692,506]),('斜十字',[1176,717]),('十字',[596,795]),('坡道',[451,490]),('电磁门',[99,480]),('路障',[1065,1018])]
    markers=[dict(name=n,x=float(world(p)[0]),y=float(world(p)[1])) for n,p in marker_specs]
    gate=world([159,480]);heading=math.pi/2
    features=[dict(kind='ramp',name='小坡道',centre=world([404,490]).tolist(),heading=-math.pi/2,length=1.2,width=.45,height=.025,
                   status='Position from drawing; height .025m assumed, visual terrain only'),
              dict(kind='obstacle',name='路障',centre=world([1065,1018]).tolist(),heading=0,length=.23,width=.10,height=.035,
                   status='Approximate footprint at road edge; dimension inferred from image')]
    targets=[world(p).tolist() for p in [[135,380],[184,603],[380,933],[596,933],[783,230],[1120,309]]]
    return dict(points=pts,tags=tags,elements=elements,markers=markers,features=features,targets=targets,
        start=[*pts[0],-math.pi/2],gate=dict(centre=gate.tolist(),heading=heading),
        source=dict(title='2026全国智能车竞赛雁过留痕组预、决赛赛道方案一（2）',image='用户提供的赛道示意图',
                    labelled_length_m=38.411,labelled_bounds_m=[6.4,5.45],scale_pixels_per_m=192,
                    reconstruction='Geometry traced using R50/L annotations and image coordinates; not the original CAD or a millimetre-accurate certified reconstruction.',
                    added_gate='On left outer long straight, near the existing ramp region but on the parallel segment. Original ramp preserved.'))
