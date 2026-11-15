from pathlib import Path
import base64,io,json,math,os,subprocess,sys,threading,time,unittest,urllib.request,urllib.error,zipfile
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from simulation import Simulation
from user_controller import prepare_project,ControllerError,ROOT
from native_core import find_compiler

def project_zip(files):
    b=io.BytesIO()
    with zipfile.ZipFile(b,'w',zipfile.ZIP_DEFLATED) as z:
        for name,body in files.items():z.writestr(name,body)
    return b.getvalue()

class CodeImportTests(unittest.TestCase):
    def setUp(self):self.sim=Simulation()
    def tearDown(self):self.sim.close()
    def test_python_controls_motion_and_reset(self):
        source=b'''def reset(config): return {'n':0}
def step(s,state):
 assert len(s['camera_gray'])==4800
 assert not {'x','y','yaw','course_progress','visual_element'} & s.keys()
 state['n']+=1
 return dict(speed_m_s=.8,steer_deg=5,fan=.4,enabled=True)
'''
        self.sim.reset(dict(course='gate_only'))
        state=self.sim.import_controller('mine.py',source)
        self.assertEqual(state['config']['controller'],'user_code')
        result=self.sim.step(100)['state']
        self.assertGreater(result['speed'],.5);self.assertGreater(result['steer_deg'],4)
        self.sim.reset(self.sim.cfg);self.assertEqual(self.sim.time,0)
        self.assertEqual(self.sim.step(1)['state']['controller_error'],'')

    def test_raw_camera_and_delayed_frame(self):
        code=b'''def step(s,state):
 if s['time_s']<.1: assert not any(s['camera_gray'])
 return dict(speed_m_s=.2,enabled=True)
'''
        self.sim.reset(dict(course='gate_only',camera_delay_ms=100))
        self.sim.import_controller('controller.py',code)
        self.assertEqual(self.sim.step(30)['state']['fault'],0)

    def test_python_exception_and_hang_stop(self):
        for code in [b'def step(s,state): raise ValueError("bad controller")',b'def step(s,state):\n while True: pass']:
            self.sim.use_reference_controller();self.sim.import_controller('controller.py',code)
            start=time.monotonic();s=self.sim.step(1)['state']
            self.assertEqual(s['fault'],7);self.assertEqual(s['enabled'],0);self.assertEqual(s['left_pwm'],0)
            self.assertTrue(s['controller_error']);self.assertLess(time.monotonic()-start,2)

    def test_nonfinite_output_stops(self):
        self.sim.import_controller('controller.py',b'def step(s,state): return {"speed_m_s":float("nan"),"enabled":True}')
        s=self.sim.step(1)['state'];self.assertEqual(s['fault'],7)

    def test_failed_import_keeps_existing_controller(self):
        self.sim.import_controller('controller.py',b'def step(s,state): return dict(enabled=True,speed_m_s=.5)')
        before=self.sim.user_controller
        with self.assertRaises(ControllerError):self.sim.import_controller('controller.py',b'this is not python!')
        self.assertIs(self.sim.user_controller,before)
        self.assertEqual(self.sim.step(1)['state']['fault'],0)

    def test_zip_project_import(self):
        z=project_zip({'src/controller.py':'from helper import speed\ndef step(s,state): return dict(speed_m_s=speed,enabled=True)','src/helper.py':'speed=.6'})
        self.sim.reset(dict(course='gate_only'));self.sim.import_controller('project.zip',z)
        self.assertGreater(self.sim.step(100)['state']['speed'],.4)

    def test_zip_paths_and_file_types(self):
        for files in [{'../escape.py':'x=1'},{'/tmp/controller.py':'x=1'},{'C:/controller.py':'x=1'},{'payload.exe':'x'},
                      {'a/controller.py':'x=1','b/controller.py':'x=1'}]:
            with self.assertRaises(ValueError):prepare_project('project.zip',project_zip(files))
        with self.assertRaises(ValueError):prepare_project('project.py',b'0'*2_000_001)

    @unittest.skipUnless(find_compiler(),'GCC/Clang required for C import')
    def test_c_sample_matches_python_sample(self):
        a=Simulation();b=Simulation()
        try:
            cfg=dict(course='gate_only',start_x=.25,speed_scale=1)
            a.reset(cfg);b.reset(cfg)
            a.import_controller('controller.c',(ROOT/'examples/controller.c').read_bytes())
            b.import_controller('controller.py',(ROOT/'examples/controller.py').read_bytes())
            for _ in range(100):a.step(1);b.step(1)
            self.assertEqual(a.telemetry(),b.telemetry())
        finally:a.close();b.close()

    @unittest.skipUnless(find_compiler(),'GCC/Clang required for C import')
    def test_c_multifile_and_compile_error(self):
        z=project_zip({'controller.c':'#include "gatecar_api.h"\nextern double speed(void);\nvoid gatecar_reset(void){}\nvoid gatecar_step(const GateCarInput*i,GateCarOutput*o){o->enabled=1;o->speed_m_s=speed();}',
                       'speed.c':'double speed(void){return .5;}'})
        self.sim.reset(dict(course='gate_only'));self.sim.import_controller('project.zip',z)
        self.assertGreater(self.sim.step(100)['state']['speed'],.3)
        old=self.sim.user_controller
        with self.assertRaises(ValueError):self.sim.import_controller('controller.c',b'broken code')
        self.assertIs(old,self.sim.user_controller)

    @unittest.skipUnless(find_compiler(),'GCC/Clang required for C import')
    def test_cpp_example(self):
        self.sim.reset(dict(course='gate_only'))
        self.sim.import_controller('controller.cpp',(ROOT/'examples/controller.c').read_bytes())
        self.assertEqual(self.sim.step(1)['state']['fault'],0)

    @unittest.skipUnless(find_compiler(),'GCC/Clang required for C import')
    def test_c_crash_does_not_crash_simulator(self):
        source=b'#include "gatecar_api.h"\n#include <stdlib.h>\nvoid gatecar_reset(void){}\nvoid gatecar_step(const GateCarInput*i,GateCarOutput*o){abort();}'
        self.sim.import_controller('controller.c',source)
        result=self.sim.step(1)['state']
        self.assertEqual(result['fault'],7);self.assertFalse(result['enabled'])
        self.assertTrue(result['controller_error'])

    def test_startup_without_compiler(self):
        with patch('native_core.find_compiler',return_value=None):
            s=Simulation()
            try:
                self.assertEqual(s.step(3)['state']['controller'],'route_demo')
                with self.assertRaises(RuntimeError):s.reset(dict(controller='native_c'))
            finally:s.close()

class HttpCodeImportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from server import SimulatorServer
        cls.http=SimulatorServer(('127.0.0.1',0));cls.thread=threading.Thread(target=cls.http.serve_forever,daemon=True);cls.thread.start()
        cls.url=f'http://127.0.0.1:{cls.http.server_port}'
    @classmethod
    def tearDownClass(cls):cls.http.shutdown();cls.http.server_close();cls.thread.join(2)
    def request(self,path,data=None,session='test-session-1',origin=None):
        headers={'X-Sim-Session':session}
        if origin:headers['Origin']=origin
        payload=None if data is None else json.dumps(data).encode();headers['Content-Type']='application/json'
        return urllib.request.urlopen(urllib.request.Request(self.url+path,data=payload,headers=headers),timeout=15)
    def test_api_import_independent_sessions_and_code_errors(self):
        code=b'def step(s,state): return dict(speed_m_s=.6,enabled=True)'
        body=dict(filename='controller.py',content_base64=base64.b64encode(code).decode())
        s=json.load(self.request('/api/controller/import',body));self.assertEqual(s['config']['controller'],'user_code')
        other=json.load(self.request('/api/state',session='test-session-2'));self.assertIsNone(other['user_controller'])
        s=json.load(self.request('/api/step',dict(ticks=30)));self.assertGreater(s['state']['speed'],0)
        self.request('/api/controller/remove',{}).close()
        self.assertIsNone(json.load(self.request('/api/state'))['user_controller'])
        with self.assertRaises(urllib.error.HTTPError) as caught:self.request('/api/controller/import',body,origin='https://example.com')
        self.assertEqual(caught.exception.code,403)
        caught.exception.close()
        with self.request('/templates/c-project.zip') as r:
            with zipfile.ZipFile(io.BytesIO(r.read())) as z:self.assertIn('controller.c',z.namelist())

if __name__=='__main__':unittest.main()
