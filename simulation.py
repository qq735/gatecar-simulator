"""Fixed-step planar vehicle and rectangular gate field, with a native C controller."""
from pathlib import Path
import math,json,csv,io,collections
import numpy as np
from native_core import NativeCore
from course import Course

DT=.005
DEFAULTS=dict(start_x=1.3,start_y=0.,start_yaw_deg=0.,speed_scale=1.6,wheelbase=.2,track_width=.145,course='room',gate_mode='all',field_mode='nearest',start_mode='lap',laps=1,
    servo_units_per_degree=3.,servo_time_ms=65,motor_time_ms=140,motor_max_speed=5.,counts_per_m=10000,
    gate_x=3.8,gate_y=0.,gate_width=.60,gate_height=.5,gap_length=.6,road_width=.45,curve=0.,
    frequency_khz=150.,gate_ampere_turns=20.,coil_turns=100,coil_area_mm2=20.,receiver_gain=8.,
    coil_forward=.379,coil_spacing=.03,coil_height=.109,coil_yaw_deg=45.,adc_zero=15.,noise_adc=2.,gain_mismatch=0.,
    mag_min_sum=40.,mag_sign=1,gate_timeout_ms=4000.,sensor_delay_ms=20.,camera_delay_ms=0.,
    camera_dropout_s=-1.,field_off_s=-1.,kill_s=-1.,seed=7,controller='route_demo',manual_speed=1.6,manual_steer_deg=0.,
    camera_h=.246,camera_pitch_deg=25.,camera_hfov_deg=70.,vision_lookahead=.32,calibration='analytic',calibration_data=[])
LIMITS=dict(start_x=(-.5,2.5),start_y=(-.4,.4),start_yaw_deg=(-25,25),speed_scale=(.2,3.5),wheelbase=(.1,.4),track_width=(.08,.3),laps=(1,3),
 servo_units_per_degree=(.5,20),servo_time_ms=(0,500),motor_time_ms=(20,1000),motor_max_speed=(.5,8),counts_per_m=(1000,100000),
 gate_x=(-1,20),gate_y=(-10,20),gate_width=(.35,1.2),gate_height=(.3,1.),gap_length=(.4,2.4),road_width=(.25,.8),curve=(0,.16),
 frequency_khz=(10,500),gate_ampere_turns=(0,200),coil_turns=(1,1000),coil_area_mm2=(1,200),receiver_gain=(1,500),
 coil_forward=(.1,.6),coil_spacing=(.01,.2),coil_height=(.02,.4),coil_yaw_deg=(10,80),adc_zero=(0,500),noise_adc=(0,300),gain_mismatch=(-.8,.8),
 mag_min_sum=(1,3000),mag_sign=(-1,1),gate_timeout_ms=(100,10000),sensor_delay_ms=(0,500),camera_delay_ms=(0,500),
 camera_dropout_s=(-1,60),field_off_s=(-1,60),kill_s=(-1,60),seed=(0,999999),manual_speed=(-2,3),manual_steer_deg=(-35,35),
 camera_h=(.1,.5),camera_pitch_deg=(10,65),camera_hfov_deg=(40,110),vision_lookahead=(.15,.8))
GATES=['循迹','入口准备','电磁引导','白道重捕获','离门确认','故障']
FAULTS=['无','未就绪','相机超时','白道丢失','磁信号失效','门区限时/限距','紧急停止','用户代码错误']

def validate(updates):
    cfg=dict(DEFAULTS)
    for k,v in (updates or {}).items():
        if k not in cfg:raise ValueError('未知参数：'+str(k))
        if k in LIMITS:
            if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v):raise ValueError('参数必须为有限数值：'+k)
            lo,hi=LIMITS[k]
            if not lo<=v<=hi:raise ValueError(f'{k} 必须在 {lo}..{hi}')
        cfg[k]=v
    if cfg['mag_sign'] not in [-1,1]:raise ValueError('mag_sign只能为-1或1')
    if cfg['controller'] not in ['native_c','sensor_reference','manual','route_demo','user_code']:raise ValueError('无效控制器')
    if cfg['course'] not in ['room','reference_2026','closed','gate_only']:raise ValueError('无效赛道')
    if cfg['gate_mode'] not in ['all','none','straight','left_ring','right_ring','omega','s_bend','hairpin','original','cross','diagonal','ramp','obstacle']:raise ValueError('无效门位')
    if cfg['field_mode'] not in ['nearest','all']:raise ValueError('无效激励模式')
    if cfg['start_mode'] not in ['lap','gate']:raise ValueError('无效起跑位置')
    if cfg['calibration'] not in ['analytic','measured']:raise ValueError('无效标定模式')
    rows=cfg['calibration_data']
    if not isinstance(rows,list) or len(rows)>5000:raise ValueError('标定数据数量不正确')
    for row in rows:
        if not isinstance(row,dict) or any(k not in row for k in ['bx_uT','by_uT','bz_uT','adc_l','adc_r']):raise ValueError('标定列需为bx_uT,by_uT,bz_uT,adc_l,adc_r')
        if any(not isinstance(v,(int,float)) or not math.isfinite(v) for v in row.values()):raise ValueError('标定数据必须是有限数值')
    if cfg['calibration']=='measured' and len(rows)<5:raise ValueError('实测标定至少需要5行数据')
    return cfg

class Simulation:
    def __init__(self,core=None):
        self.core=core or NativeCore();self.user_controller=None;self.controller_error='';self.reset()
    def reset(self,updates=None):
        candidate=validate(updates)
        if candidate['controller']=='user_code' and self.user_controller is None:raise ValueError('先导入用户控制代码，再选择用户代码模式')
        self.cfg=candidate;c=self.cfg;self.rng=np.random.default_rng(int(c['seed']));self.tick=0;self.controller_error=''
        self.x=c['start_x'];self.y=c['start_y'];self.yaw=math.radians(c['start_yaw_deg']);self.vl=0.;self.vr=0.;self.steer=0.;self.yawrate=0.
        self.output={k:0 for k in ['steer','left_pwm','right_pwm','fan_command','target_l','target_r','mag_error','mag_valid','enabled','element','gate','fault','finish_count','finished']}
        self.output['steer']=2975;self.status='ready';self.events=[];self.history=[];self.trace=[];self.has_guided=False;self.passed_gate=False;self.clear_seen=False
        self.adc_queue=collections.deque();self.camera_queue=collections.deque();self.current_frame=dict(error=0,line=True,gap=False,mask=[])
        if c['course']=='room':
            from room_course import RoomCourse
            self.course=RoomCourse(c['gate_mode'],c['gate_width'],c['gap_length'])
        else:self.course=Course(c['course'],c['gate_x'])
        if self.course.start is not None:
            sx,sy,heading=self.course.start;self.x=float(sx-c['start_y']*math.sin(heading));self.y=float(sy+c['start_y']*math.cos(heading));self.yaw=heading+math.radians(c['start_yaw_deg'])
        self.gates=getattr(self.course,'gates',[dict(self.course.gate,id='single',name='电磁门')])
        self.local_end_progress=None
        if c['course']=='room' and c['start_mode']=='gate' and self.gates:
            g=self.gates[0];begin=max(0,g['progress']-1.0);position,heading=self.course.pose(begin)
            self.x=float(position[0]-c['start_y']*math.sin(heading));self.y=float(position[1]+c['start_y']*math.cos(heading));self.yaw=heading+math.radians(c['start_yaw_deg'])
            self.local_end_progress=g['progress']+1.0
        self.fields=[RectangularGate(*g['centre'],c['gate_width'],c['gate_height'],c['gate_ampere_turns'],heading=g['heading']) for g in self.gates]
        self.field=self.fields[0] if self.fields else RectangularGate(0,0,c['gate_width'],c['gate_height'],0)
        self.progress,self.road_distance,self.route_index=self.course.locate([self.x,self.y]);self.start_progress=self.progress;self.element_label='直道';self.element_code=0
        self.core.reset(c)
        if c['controller']=='user_code':self.user_controller.reset(c)
        self.camera_mask=None;self.previous_gate=0;self.reference_gate=0;self.field_sample=np.zeros(3);self.last_adc=[0,0]
        self.gate_passes=[];self.active_gate_index=0;self.minimum_gate_clearance=1.;self.max_speed_seen=0.
        self.calibration_fit=None
        if c['calibration']=='measured':
            rows=c['calibration_data'];a=np.array([[r['bx_uT'],r['by_uT'],r['bz_uT'],1] for r in rows]);b=np.array([[r['adc_l'],r['adc_r']] for r in rows]);self.calibration_fit=np.linalg.lstsq(a,b,rcond=None)[0]
        self.camera_geometry();return self.snapshot()
    @property
    def time(self):return self.tick*DT
    def road_y(self,x):
        x=np.asarray(x);return self.cfg['curve']*np.sin(np.maximum(x-5.0,0)*1.8)
    def camera_geometry(self):
        c=self.cfg;w,h=80,60;f=w/(2*math.tan(math.radians(c['camera_hfov_deg'])/2));pitch=math.radians(c['camera_pitch_deg'])
        cols,rows=np.meshgrid(np.arange(w)+.5,np.arange(h)+.5);u=(cols-w/2)/f;v=(rows-h/2)/f
        dx=math.cos(pitch)-v*math.sin(pitch);dy=-u;dz=-math.sin(pitch)-v*math.cos(pitch)
        scale=np.where(dz<-.01,-c['camera_h']/dz,np.nan)
        self.camera_ground=np.stack([dx*scale+.145,dy*scale],axis=2)
    def capture_camera(self):
        c=self.cfg;p=self.camera_ground;cs=math.cos(self.yaw);sn=math.sin(self.yaw)
        gx=self.x+p[:,:,0]*cs-p[:,:,1]*sn;gy=self.y+p[:,:,0]*sn+p[:,:,1]*cs
        visible=self.course.is_road(np.stack([gx,gy],axis=2),c['road_width'],c['gap_length'])
        mask=np.where(visible,240,24).astype(np.uint8)
        # Estimate visual pursuit from the row of the synthetic mask nearest lookahead.
        distances=self.camera_ground[:,40,0];idx=int(np.nanargmin(np.abs(distances-c['vision_lookahead'])))
        good=np.where(mask[idx]>128)[0]
        valid=len(good)>4
        if valid:
            local_y=float(np.mean(p[idx,good,1]));look=float(np.mean(p[idx,good,0]));error=math.degrees(math.atan2(local_y,max(.05,look)))
        else:error=0.
        far=int(np.nanargmin(np.abs(distances-(c['vision_lookahead']+.38))))
        gap_hint=bool(np.sum(mask[far]>128)<5 and np.sum(mask[idx]>128)>5)
        self.camera_mask=mask
        self.camera_queue.append((self.time+c['camera_delay_ms']/1000,dict(error=max(-30,min(30,error)),line=bool(valid),gap=gap_hint,mask=mask.reshape(-1).tolist())))
    def sensor_adc(self):
        c=self.cfg;cs=math.cos(self.yaw);sn=math.sin(self.yaw);forward=c['coil_forward']
        readings=[];fields=[]
        for side in [1,-1]:
            lateral=side*c['coil_spacing']/2;p=np.array([self.x+cs*forward-sn*lateral,self.y+sn*forward+cs*lateral,c['coil_height']])
            if not self.fields:b=np.zeros(3)
            elif c['field_mode']=='all':b=sum((f.sample(p) for f in self.fields),start=np.zeros(3))
            else:b=self.fields[self.active_gate_index].sample(p)
            if c['field_off_s']>=0 and self.time>=c['field_off_s']:b=b*0
            fields.append(b)
            a=self.yaw+side*math.radians(c['coil_yaw_deg']);axis=np.array([math.cos(a),math.sin(a),0])
            voltage=2*math.pi*c['frequency_khz']*1000*c['coil_turns']*c['coil_area_mm2']*1e-6*abs(float(b@axis))*c['receiver_gain']
            adc=voltage/3.3*4095*(1+side*c['gain_mismatch'])+c['adc_zero']
            readings.append(adc)
        self.field_sample=np.mean(fields,axis=0)
        if self.calibration_fit is not None:
            # Fitted measured envelope mapping is local linear, valid only near sampled field range.
            readings=np.append(self.field_sample*1e6,1)@self.calibration_fit
        readings=np.clip(np.asarray(readings)+self.rng.normal(0,c['noise_adc'],2),0,4095).round().astype(int)
        self.adc_queue.append((self.time+c['sensor_delay_ms']/1000,readings.tolist()))
        while self.adc_queue and self.adc_queue[0][0]<=self.time+1e-8:self.last_adc=self.adc_queue.popleft()[1]
        return self.last_adc
    def update_active_gate(self):
        if not self.gates:return
        if self.cfg['course']=='room':
            # Route progress is scene truth used for scheduling excitation only.
            diffs=np.array([g['progress']-self.progress%self.course.length for g in self.gates])
            diffs=np.where(diffs<-.7,diffs+self.course.length,diffs)
            self.active_gate_index=int(np.argmin(abs(diffs)))
        else:self.active_gate_index=0
        self.field=self.fields[self.active_gate_index]
    def step(self,n=10):
        n=max(1,min(200,int(n)));samples=[]
        for _ in range(n):
            if self.status in ['passed','collision','off_track','stopped','timeout']:break
            c=self.cfg;self.status='running';new_frame=False
            if self.tick%4==0 and not(c['camera_dropout_s']>=0 and self.time>=c['camera_dropout_s']):self.capture_camera()
            while self.camera_queue and self.camera_queue[0][0]<=self.time+1e-8:
                self.current_frame=self.camera_queue.popleft()[1];new_frame=True
            self.update_active_gate();adc=self.sensor_adc();frame=self.current_frame
            # User projects receive calibrated encoder counts without legacy speed remapping.
            cp=c['counts_per_m'] if c['controller']=='user_code' else c['counts_per_m']/c['speed_scale']
            i=dict(adc_l=adc[0],adc_r=adc[1],wheel_l=self.vl*DT*cp,wheel_r=self.vr*DT*cp,gyro_z=-math.degrees(self.yawrate),
                visual_error=frame['error'],visual_element=self.element_code,frame_new=int(new_frame),line_valid=int(frame['line']),gap_hint=int(frame['gap']),arm=int(self.tick==0),
                kill=int(c['kill_s']>=0 and self.time>=c['kill_s']),finish_mark=0)
            if c['controller']=='user_code':o=self.user_step(i)
            elif c['controller']=='native_c':o=self.core.step(i)
            else:o=self.reference(i)
            self.output=o
            if o['gate']!=self.previous_gate:
                self.events.append(dict(t=round(self.time,3),event=GATES[o['gate']],fault=FAULTS[o['fault']]));self.previous_gate=o['gate']
            if o['gate']==2:self.has_guided=True
            if self.has_guided and o['gate']==4:self.clear_seen=True
            if o['fault']:self.status='stopped';self.events.append(dict(t=round(self.time,3),event='故障锁存',fault=FAULTS[o['fault']]))
            desired=math.radians(max(-35,min(35,(o['steer']-2975)/c['servo_units_per_degree'])))
            alpha=1 if c['servo_time_ms']==0 else min(1,DT/(c['servo_time_ms']/1000))
            self.steer+=(desired-self.steer)*alpha
            old_v=(self.vl+self.vr)/2
            for attr,key in [('vl','left_pwm'),('vr','right_pwm')]:
                v=getattr(self,attr);target=o[key]/10000*c['motor_max_speed'] if o['enabled'] else 0
                dv=np.clip((target-v)*DT/(c['motor_time_ms']/1000),-DT*8,DT*5);setattr(self,attr,float(v+dv))
            v=(self.vl+self.vr)/2;self.yawrate=v/c['wheelbase']*math.tan(self.steer)
            self.x+=v*math.cos(self.yaw)*DT;self.y+=v*math.sin(self.yaw)*DT;self.yaw+=self.yawrate*DT;self.tick+=1
            self.progress,self.road_distance,self.route_index=self.course.locate([self.x,self.y],self.progress)
            self.element_label,self.element_code=self.course.element(self.progress,self.route_index)
            # Gate plane collision checks the swept vehicle centre/half-width against opening.
            for gate_index,g in enumerate(self.gates):
                tangent=np.array([math.cos(g['heading']),math.sin(g['heading'])]);normal=np.array([-tangent[1],tangent[0]])
                delta=np.array([self.x,self.y])-np.array(g['centre']);gate_u=float(delta@tangent);gate_v=float(delta@normal)
                previous_delta=delta-np.array([math.cos(self.yaw),math.sin(self.yaw)])*v*DT
                previous_u=float(previous_delta@tangent)
                if abs(gate_u)<=.10 and abs(gate_v)<.7:
                    relative=self.yaw-g['heading'];extent=.095*abs(math.cos(relative))+.20*abs(math.sin(relative))
                    clearance=c['gate_width']/2-abs(gate_v)-extent
                    if abs(gate_v)<c['gate_width']/2:self.minimum_gate_clearance=min(self.minimum_gate_clearance,clearance)
                    if clearance<0 and abs(gate_v)-extent<c['gate_width']/2+.025:
                        self.status='collision';self.events.append(dict(t=round(self.time,3),event=g.get('name','电磁门')+'门框碰撞',fault='车身外包络越界'))
                if previous_u<=0<gate_u and abs(gate_v)<c['gate_width']/2:
                    self.passed_gate=True
                    expected_lap=math.floor((self.progress-self.start_progress)/self.course.length)
                    key=(gate_index,expected_lap)
                    if key not in self.gate_passes:
                        self.gate_passes.append(key);self.events.append(dict(t=round(self.time,3),event='通过 '+g.get('name','电磁门'),fault=''))
            active=self.gates[self.active_gate_index] if self.gates else self.course.gate
            tangent=np.array([math.cos(active['heading']),math.sin(active['heading'])]);normal=np.array([-tangent[1],tangent[0]])
            delta=np.array([self.x,self.y])-np.array(active['centre']);gate_u=float(delta@tangent);gate_v=float(delta@normal)
            self.max_speed_seen=max(self.max_speed_seen,v)
            if self.cfg['course']=='gate_only' and gate_u>c['gap_length']/2+.8 and self.passed_gate:
                self.status='passed';self.events.append(dict(t=round(self.time,3),event='完成穿门与出口段',fault=''))
            if self.cfg['course']!='gate_only' and self.progress-self.start_progress>=self.course.length*int(c['laps']):
                self.status='passed';self.events.append(dict(t=round(self.time,3),event='完成闭环整圈',fault=''))
            if self.local_end_progress is not None and self.progress>=self.local_end_progress:
                self.status='passed';self.events.append(dict(t=round(self.time,3),event='完成单门出口测试段',fault=''))
            if self.road_distance>1.:self.status='off_track'
            if self.time>=180:self.status='timeout'
            row=self.telemetry();self.history.append(row)
            if self.tick%4==0:self.trace.append([self.x,self.y]);samples.append(row)
        return dict(state=self.snapshot(),samples=samples)
    def user_step(self,i):
        c=self.cfg
        sensors=dict(dt_s=DT,time_s=self.time,wheelbase_m=c['wheelbase'],track_width_m=c['track_width'],
          counts_per_m=c['counts_per_m'],motor_max_speed_m_s=c['motor_max_speed'],speed_scale=c['speed_scale'],
          adc_zero=c['adc_zero'],mag_min_sum=c['mag_min_sum'],adc_l=int(i['adc_l']),adc_r=int(i['adc_r']),
          wheel_l=int(round(i['wheel_l'])),wheel_r=int(round(i['wheel_r'])),gyro_z_deg_s=float(i['gyro_z']),
          visual_error_deg=float(i['visual_error']),frame_new=int(i['frame_new']),line_valid=int(i['line_valid']),
          gap_hint=int(i['gap_hint']),arm=int(i['arm']),kill=int(i['kill']),camera_gray=self.current_frame.get('mask') or [0]*4800)
        try:
            r=self.user_controller.step(sensors)
            speed=max(-2.,min(c['motor_max_speed'],float(r['speed_m_s'])))
            duty=int(round(speed/c['motor_max_speed']*10000))
            fault=int(r['fault']);enabled=bool(r['enabled']) and not fault and not i['kill']
            if i['kill']:fault=6
            left=int(max(-10000,min(10000,r['left_pwm']))) if r['use_pwm'] else duty
            right=int(max(-10000,min(10000,r['right_pwm']))) if r['use_pwm'] else duty
            angle=max(-35.,min(35.,float(r['steer_deg'])))
            return dict(steer=2975+angle*c['servo_units_per_degree'],left_pwm=left if enabled else 0,right_pwm=right if enabled else 0,
              fan_command=500+int(max(0.,min(1.,r['fan']))*500) if enabled else 500,target_l=0,target_r=0,
              mag_error=0,mag_valid=int(i['adc_l']+i['adc_r']-2*c['adc_zero']>=c['mag_min_sum']),enabled=int(enabled),
              element=0,gate=5 if fault else int(r['gate_state']),fault=fault,finish_count=0,finished=0)
        except Exception as exc:
            self.controller_error=str(exc);self.user_controller.close()
            return dict(steer=2975,left_pwm=0,right_pwm=0,fan_command=500,target_l=0,target_r=0,mag_error=0,mag_valid=0,
                        enabled=0,element=0,gate=5,fault=7,finish_count=0,finished=0)

    def import_controller(self,filename,content):
        from user_controller import prepare_project
        candidate=prepare_project(filename,content);settings=dict(self.cfg,controller='user_code')
        try:candidate.reset(settings)
        except Exception:candidate.close();raise
        old=self.user_controller;self.user_controller=candidate
        if old:old.close()
        return self.reset(settings)

    def use_reference_controller(self):
        if self.user_controller:self.user_controller.close();self.user_controller=None
        return self.reset(dict(self.cfg,controller='route_demo'))

    def close(self):
        if self.user_controller:self.user_controller.close()
        self.core.close()

    def reference(self,i):
        c=self.cfg;sumadc=max(1,i['adc_l']+i['adc_r']-2*c['adc_zero']);valid=sumadc>=c['mag_min_sum'] and max(i['adc_l'],i['adc_r'])<4000
        e=max(-1.,min(1.,(i['adc_l']-i['adc_r'])/sumadc*c['mag_sign'])) if valid else 0.
        if i['kill']:fault=6
        elif not i['line_valid'] and not valid:fault=4
        else:fault=0
        if i['gap_hint'] and valid:self.reference_gate=1
        if not i['line_valid'] and self.reference_gate:self.reference_gate=2
        if self.reference_gate==2 and i['line_valid'] and self.x>c['gate_x']:self.reference_gate=4
        error=i['visual_error'] if i['line_valid'] else 30*e
        if c['controller']=='manual':speed=c['manual_speed'];angle=c['manual_steer_deg']
        elif c['controller']=='route_demo':
            # Explicit ideal route navigation for inspecting the complete course.
            # It uses course truth, and is never labelled as testing the C controller.
            lookahead=.22 if c['course']=='room' else .26 if c['course']=='reference_2026' else .38
            target=self.course.target(self.progress,lookahead)
            dx,dy=target-np.array([self.x,self.y]);local_y=-math.sin(self.yaw)*dx+math.cos(self.yaw)*dy
            dist2=max(.03,dx*dx+dy*dy);angle=math.degrees(math.atan2(2*c['wheelbase']*local_y,dist2))
            if c['course']=='room':
                # Higher physical speed with curvature-aware corner limiting, not playback acceleration.
                p0,h0=self.course.pose(self.progress+.08);p1,h1=self.course.pose(self.progress+.24)
                curvature=abs(math.atan2(math.sin(h1-h0),math.cos(h1-h0)))/.16
                speed=min(1.35*c['speed_scale'],math.sqrt(3.0/max(.15,curvature)))
            else:speed=(.80 if self.element_label in ['直道','门区直道','坡道直道','路障直道','十字直道'] else .52)*c['speed_scale'] if c['course']=='reference_2026' else (1.15 if self.element_label=='直道' else .85)*c['speed_scale']
            fault=6 if i['kill'] else 0
            gate=self.gates[self.active_gate_index] if self.gates else self.course.gate
            tangent=np.array([math.cos(gate['heading']),math.sin(gate['heading'])]);normal=np.array([-tangent[1],tangent[0]])
            delta=np.array([self.x,self.y])-np.array(gate['centre']);u=float(delta@tangent);v=float(delta@normal)
            if self.gates and abs(v)<.3 and -c['gap_length']/2-.4<u<c['gap_length']/2:
                self.reference_gate=2
            elif self.reference_gate==2:self.reference_gate=4
            elif self.reference_gate==4 and (u>c['gap_length']/2+.8 or abs(v)>.3):self.reference_gate=0
        else:speed=1.2*c['speed_scale'] if self.reference_gate==0 else .65*c['speed_scale'];angle=error*1.8
        return dict(steer=2975+angle*c['servo_units_per_degree'],left_pwm=int(speed/c['motor_max_speed']*10000),right_pwm=int(speed/c['motor_max_speed']*10000),fan_command=775,
            target_l=0,target_r=0,mag_error=int(e*1000),mag_valid=int(valid),enabled=int(fault==0),element=0,gate=5 if fault else self.reference_gate,fault=fault,finish_count=0,finished=0)
    def telemetry(self):
        return dict(t=round(self.time,4),x=round(self.x,5),y=round(self.y,5),yaw_deg=round(math.degrees(self.yaw),3),speed=round((self.vl+self.vr)/2,4),
            steer_deg=round(math.degrees(self.steer),3),adc_l=int(self.last_adc[0]),adc_r=int(self.last_adc[1]),bx_uT=round(float(self.field_sample[0]*1e6),5),by_uT=round(float(self.field_sample[1]*1e6),5),bz_uT=round(float(self.field_sample[2]*1e6),5),
            visual_error=round(self.current_frame['error'],2),line_valid=int(self.current_frame['line']),gap_hint=int(self.current_frame['gap']),course_progress=round(self.progress,4),course_element=self.element_label,
            active_gate=self.gates[self.active_gate_index].get('name','电磁门') if self.gates else '无',passed_gates=len(self.gate_passes),**self.output)
    def snapshot(self):
        terrain_height=float(self.course.height(np.array([self.x,self.y])))
        ahead=np.array([self.x,self.y])+np.array([math.cos(self.yaw),math.sin(self.yaw)])*.03
        behind=np.array([self.x,self.y])-np.array([math.cos(self.yaw),math.sin(self.yaw)])*.03
        slope=float(self.course.height(ahead)-self.course.height(behind))/.06
        return dict(**self.telemetry(),status=self.status,controller=self.cfg['controller'],gate_label=GATES[self.output['gate']],fault_label=FAULTS[self.output['fault']],
            terrain_visual_height=terrain_height,terrain_visual_pitch_deg=math.degrees(math.atan(slope)),
            trace=self.trace[-6000:],events=self.events[-25:],camera=self.camera_mask.tolist() if self.camera_mask is not None else None,
            passed_plane=self.passed_gate,saw_magnetic_guidance=self.has_guided,saw_exit_confirmation=self.clear_seen,
            lap_progress=max(0,(self.progress-self.start_progress)/self.course.length),course_length=self.course.length,
            gate_count=len(self.gates),max_speed=self.max_speed_seen,minimum_gate_clearance=self.minimum_gate_clearance,
            active_gate_index=self.active_gate_index,local_gate_test=self.local_end_progress is not None,
            user_controller=self.user_controller.manifest if self.user_controller else None,controller_error=self.controller_error,config=self.cfg)
    def csv(self):
        b=io.StringIO();writer=csv.DictWriter(b,fieldnames=list(self.telemetry()));writer.writeheader();writer.writerows(self.history);return b.getvalue()
    def field_grid(self):
        data=[]
        for gate,field in zip(self.gates,self.fields):
            centre=np.array(gate['centre']);tangent=np.array([math.cos(gate['heading']),math.sin(gate['heading'])]);normal=np.array([-tangent[1],tangent[0]])
            for u in np.linspace(-.7,.7,11):
                for v in np.linspace(-.4,.4,7):
                    x,y=centre+u*tangent+v*normal
                    b=field.sample([x,y,self.cfg['coil_height']])*1e6;data.append([float(x),float(y),float(b[0]),float(b[1]),float(np.linalg.norm(b))])
        return data

class RectangularGate:
    def __init__(self,x,y,width,height,ampere_turns,segments=40,heading=0.):
        corners=np.array([[x,y-width/2,0],[x,y+width/2,0],[x,y+width/2,height],[x,y-width/2,height],[x,y-width/2,0]])
        normal=np.array([-math.sin(heading),math.cos(heading)])
        corners=np.array([[x+sign*width/2*normal[0],y+sign*width/2*normal[1],z] for sign,z in [(-1,0),(1,0),(1,height),(-1,height),(-1,0)]])
        points=[];dl=[]
        for a,b in zip(corners[:-1],corners[1:]):
            d=(b-a)/segments
            for i in range(segments):points.append(a+(i+.5)*d);dl.append(d)
        self.mid=np.array(points);self.dl=np.array(dl);self.current=ampere_turns
    def sample(self,p):
        r=np.asarray(p)-self.mid;distance=np.maximum(np.linalg.norm(r,axis=1),.003)
        return 1e-7*self.current*np.sum(np.cross(self.dl,r)/distance[:,None]**3,axis=0)
