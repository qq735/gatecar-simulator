"""Geometry, vehicle speed, local gate scenarios and C-core fault regression."""
from pathlib import Path
import json,math,time
import numpy as np
from simulation import Simulation,validate
from native_core import NativeCore
from room_course import RoomCourse

ROOT=Path(__file__).parent;core=NativeCore();results=[]
def run(cfg):
    sim=Simulation(core.fork());sim.reset(cfg)
    while sim.status in ['ready','running']:sim.step(200)
    state=sim.snapshot()
    return sim,state

c=RoomCourse()
assert abs(c.length-30.33116463857463)<.001
assert len(c.gates)==11
assert {'left_ring','right_ring','omega','s_bend','hairpin','cross','diagonal','ramp','obstacle'}.issubset({g['id'] for g in c.gates})
for g in c.gates:
    assert not c.is_road(np.array([g['centre']]),.45,.6)[0]
    assert g['foot_clearance_mm']>=12
    if 'ring' in g['id']:
        before,_=c.pose(g['progress']-.2);after,_=c.pose(g['progress']+.2)
        assert np.linalg.norm((before+after)/2-np.array(g['centre']))>.025,'Curved gap must follow actual arc'
results.append(dict(test='room_geometry_and_11_curved_gate_gaps',pass_=True,catalog=c.gate_catalog))

speeds={}
for scale in [1.,1.6,2.2]:
    sim,s=run(dict(course='room',controller='route_demo',speed_scale=scale))
    assert s['status']=='passed',(scale,s['status'],s['events'])
    assert s['passed_gates']==11,(scale,s['passed_gates'])
    assert s['minimum_gate_clearance']>0
    residual=sim.course.tree.query(np.array([[r['x'],r['y']] for r in sim.history]))[0]
    assert residual.max()<.225,(scale,residual.max())
    speeds[scale]=s
    results.append(dict(test='multi_gate_ideal_demo_speed_'+str(scale),status=s['status'],time=s['t'],max_speed=s['max_speed'],passed_gates=s['passed_gates'],minimum_opening_clearance_m=s['minimum_gate_clearance'],claim='Ideal-route vehicle model only, not native C completion'))
    (ROOT/'build'/f'multigate_speed_{scale}.csv').write_text(sim.csv(),encoding='utf-8-sig')
    print(json.dumps(results[-1],ensure_ascii=False),flush=True)
assert speeds[1.6]['max_speed']>speeds[1.]['max_speed']*1.3
assert speeds[1.6]['t']<speeds[1.]['t']

for ident in [g['id'] for g in c.gates]:
    sim,s=run(dict(course='room',controller='route_demo',gate_mode=ident,start_mode='gate'))
    assert s['status']=='passed' and s['passed_gates']==1,(ident,s['status'],s['passed_gates'],s['events'])
    results.append(dict(test='local_gate_'+ident,status=s['status'],time=s['t']))
print('LOCAL_GATES_PASS',11,flush=True)

# Two independent runs, batched differently, must have exactly identical 5ms states.
a=Simulation(core.fork());b=Simulation(core.fork())
for _ in range(200):a.step(1)
b.step(200)
assert a.telemetry()==b.telemetry()
results.append(dict(test='fixed_step_batching',pass_=True))

# Original single-gate C-controller behavior remains explicit and is not replaced by ideal pursuit.
base=dict(course='gate_only',controller='native_c',start_x=.25,speed_scale=1.,gap_length=1.2,gate_width=.55)
for name,updates,fault in [('nominal',{},0),('kill',dict(kill_s=2),6),('camera_timeout',dict(camera_dropout_s=2),2),('field_loss',dict(field_off_s=3),4)]:
    sim,s=run(dict(base,**updates))
    assert s['fault']==fault,(name,s['fault'],s['status'])
    if not fault:assert s['status']=='passed' and s['saw_exit_confirmation']
    else:assert s['status']=='stopped' and s['enabled']==0 and s['left_pwm']==s['right_pwm']==0
    results.append(dict(test='native_C_'+name,status=s['status'],fault=s['fault_label'],time=s['t']))

for ident in ['left_ring','right_ring','omega','cross']:
    sim,s=run(dict(course='room',controller='native_c',gate_mode=ident,start_mode='gate',speed_scale=1.))
    results.append(dict(test='native_C_curve_observation_'+ident,status=s['status'],fault=s['fault_label'],time=s['t'],passed_gates=s['passed_gates'],claim='Observed result, no requirement to pass unsupported complete vision path'))
    print(json.dumps(results[-1],ensure_ascii=False),flush=True)

for bad in [dict(gate_mode='bad'),dict(start_mode='bad'),dict(field_mode='bad'),dict(speed_scale=4)]:
    try:validate(bad)
    except ValueError:pass
    else:raise AssertionError(bad)
report=dict(tests=results,native_source=core.manifest,claim='Software-in-loop and static geometry only; no measured magnetic hardware or real vehicle-speed claim.')
(ROOT/'build'/'multigate_test_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print('MULTIGATE_TESTS_COMPLETE',len(results),flush=True)
