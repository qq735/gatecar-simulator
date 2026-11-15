"""Import portable Python/C controllers, execute in a bounded worker process."""
from pathlib import Path, PurePosixPath
import base64, ctypes, hashlib, importlib.util, io, json, math, multiprocessing as mp
import os, subprocess, sys, time, zipfile
from native_core import ROOT, find_compiler, shared_compile_command

MAX_UPLOAD = 2_000_000
MAX_EXPANDED = 8_000_000

class ControllerError(RuntimeError): pass

class CInput(ctypes.Structure):
    _fields_=[('api_version',ctypes.c_int32),('struct_size',ctypes.c_int32)]+[(k,ctypes.c_double) for k in
      ['dt_s','time_s','wheelbase_m','track_width_m','counts_per_m','motor_max_speed_m_s','speed_scale','adc_zero','mag_min_sum']]+[(k,ctypes.c_int32) for k in ['adc_l','adc_r','wheel_l','wheel_r']]+[(k,ctypes.c_double) for k in ['gyro_z_deg_s','visual_error_deg']]+[(k,ctypes.c_int32) for k in ['frame_new','line_valid','gap_hint','arm','kill']]+[('camera_gray',ctypes.c_uint8*4800)]

class COutput(ctypes.Structure):
    _fields_=[(k,ctypes.c_double) for k in ['speed_m_s','steer_deg','fan']]+[(k,ctypes.c_int32) for k in ['enabled','use_pwm','left_pwm','right_pwm','gate_state','fault']]

def _worker(conn,kind,path):
    """Contain crashes and infinite loops. This process is NOT a security sandbox."""
    try:
        if kind=='python':
            spec=importlib.util.spec_from_file_location('gatecar_user_code',path)
            module=importlib.util.module_from_spec(spec);sys.path.insert(0,str(Path(path).parent));spec.loader.exec_module(module)
            if not callable(getattr(module,'step',None)): raise ValueError('Python代码需要step(sensors, state)函数')
        else:
            lib=ctypes.CDLL(path)
            for name in ['gatecar_api_version','gatecar_input_size','gatecar_output_size']:getattr(lib,name).restype=ctypes.c_int
            if lib.gatecar_api_version()!=1 or lib.gatecar_input_size()!=ctypes.sizeof(CInput) or lib.gatecar_output_size()!=ctypes.sizeof(COutput):raise ValueError('C接口版本/结构体大小不匹配，请使用包内gatecar_api.h')
            lib.gatecar_reset.argtypes=[];lib.gatecar_reset.restype=None
            lib.gatecar_step.argtypes=[ctypes.POINTER(CInput),ctypes.POINTER(COutput)];lib.gatecar_step.restype=None
        state=None;conn.send(('ready',None))
        while True:
            op,payload=conn.recv()
            if op=='close': break
            if op=='reset':
                if kind=='python':state=module.reset(payload) if callable(getattr(module,'reset',None)) else {}
                else:lib.gatecar_reset()
                conn.send(('ok',None))
            elif op=='step':
                if kind=='python':result=module.step(payload,state)
                else:
                    i=CInput();i.api_version=1;i.struct_size=ctypes.sizeof(CInput)
                    for k,_ in CInput._fields_:
                        if k in ['api_version','struct_size','camera_gray']:continue
                        setattr(i,k,payload[k])
                    i.camera_gray[:]=payload['camera_gray'];o=COutput();lib.gatecar_step(ctypes.byref(i),ctypes.byref(o))
                    result={k:getattr(o,k) for k,_ in COutput._fields_}
                conn.send(('ok',result))
    except (EOFError,BrokenPipeError): pass
    except BaseException as e:
        try:conn.send(('error',type(e).__name__+': '+str(e)[:3000]))
        except (BrokenPipeError,EOFError):pass
    finally:conn.close()

class UserController:
    def __init__(self,kind,path,manifest,step_timeout=.1):
        self.kind=kind;self.path=str(path);self.manifest=manifest;self.step_timeout=step_timeout
        context=mp.get_context('spawn');self.conn,child=context.Pipe()
        self.process=context.Process(target=_worker,args=(child,kind,self.path),daemon=True)
        self.process.start();child.close()
        try:self._receive(8,'ready')
        except Exception:self.close();raise

    def _receive(self,timeout,expected='ok'):
        try:
            if not self.conn.poll(timeout):raise ControllerError('用户代码响应超时，已停止测试')
            status,value=self.conn.recv()
            if status!=expected:raise ControllerError(str(value))
            return value
        except (EOFError,OSError) as e:raise ControllerError('用户代码进程已退出，可能崩溃或访问了无效内存') from e

    def _call(self,operation,payload,timeout):
        try:self.conn.send((operation,payload));return self._receive(timeout)
        except Exception:self.close();raise

    def reset(self,config):
        keys=['wheelbase','track_width','counts_per_m','motor_max_speed','speed_scale','adc_zero','mag_min_sum']
        return self._call('reset',{k:config[k] for k in keys},2)

    def step(self,sensors):
        raw=self._call('step',sensors,self.step_timeout)
        if not isinstance(raw,dict):raise ControllerError('step返回值必须是字典')
        result=dict(speed_m_s=0.,steer_deg=0.,fan=0.,enabled=False,use_pwm=False,left_pwm=0,right_pwm=0,gate_state=0,fault=0)
        for k,v in raw.items():
            if k not in result:continue
            if not isinstance(v,(int,float,bool)) or not math.isfinite(v):raise ControllerError('输出必须为有限数值：'+k)
            result[k]=v
        if not 0<=result['gate_state']<=4 or int(result['gate_state'])!=result['gate_state']:raise ControllerError('gate_state需要是0..4整数')
        if not 0<=result['fault']<=6 or int(result['fault'])!=result['fault']:raise ControllerError('fault需要是0..6整数')
        return result

    def close(self):
        if getattr(self,'process',None) is not None:
            if self.process.is_alive():
                try:self.conn.send(('close',None))
                except (OSError,EOFError,BrokenPipeError):pass
                self.process.join(.15)
                if self.process.is_alive():self.process.terminate();self.process.join(1)
            self.conn.close();self.process=None

def prepare_project(filename,content):
    filename=Path(filename.replace('\\','/')).name
    if not filename or len(content)>MAX_UPLOAD:raise ValueError('代码文件为空或超过2 MB')
    root=ROOT/'build'/'user_projects'/str(time.time_ns());root.mkdir(parents=True)
    if filename.lower().endswith('.zip'):
        with zipfile.ZipFile(io.BytesIO(content)) as z:
            infos=z.infolist()
            if len(infos)>100 or sum(i.file_size for i in infos)>MAX_EXPANDED:raise ValueError('ZIP展开后超过8 MB或100个文件')
            for item in infos:
                path=PurePosixPath(item.filename.replace('\\','/'))
                if path.is_absolute() or '..' in path.parts or ':' in str(path):raise ValueError('ZIP内含不允许的路径')
                if item.is_dir():continue
                if Path(path.name).suffix.lower() not in ['.c','.h','.cpp','.hpp','.py','.txt','.md','.json']:raise ValueError('ZIP只接受源码、头文件和文档')
                dest=root.joinpath(*path.parts);dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(z.read(item))
    elif Path(filename).suffix.lower() in ['.py','.c','.cpp']:(root/filename).write_bytes(content)
    else:raise ValueError('请选择.py、.c、.cpp或源码ZIP')
    py=list(root.rglob('controller.py'));c=list(root.rglob('controller.c'));cpp=list(root.rglob('controller.cpp'))
    if filename.lower().endswith('.py'):py=[root/filename]
    elif filename.lower().endswith('.c'):c=[root/filename]
    elif filename.lower().endswith('.cpp'):cpp=[root/filename]
    entries=py+c+cpp
    if len(entries)!=1:raise ValueError('项目中需要唯一的controller.py、controller.c或controller.cpp入口')
    entry=entries[0];kind='python' if entry.suffix=='.py' else 'c'
    manifest=dict(name=filename,language=kind,api_version=1,sha256=hashlib.sha256(content).hexdigest(),file_count=sum(1 for p in root.rglob('*') if p.is_file()))
    if kind=='c':
        compiler=find_compiler()
        if not compiler:raise ValueError('导入C/C++需要GCC或Clang。Python控制器可直接使用。')
        sources=sorted(list(root.rglob('*.c'))+list(root.rglob('*.cpp')))
        if len(sources)>30:raise ValueError('C/C++源文件超过30个')
        if any(p.name=='abi.c' for p in sources):raise ValueError('abi.c由仿真提供，请不要在上传包内重复放入')
        # Use the package's API definition so host and plugin cannot silently disagree.
        for p in root.rglob('gatecar_api.h'):p.write_bytes((ROOT/'sdk/gatecar_api.h').read_bytes())
        (entry.parent/'gatecar_api.h').write_bytes((ROOT/'sdk/gatecar_api.h').read_bytes())
        output=root/('controller.dll' if os.name=='nt' else 'controller.dylib' if sys.platform=='darwin' else 'controller.so')
        cmd=shared_compile_command(compiler,[ROOT/'sdk/abi.c',*sources],sorted({ROOT/'sdk',entry.parent,*[p.parent for p in sources]}),output,bool(cpp or any(p.suffix=='.cpp' for p in sources)))
        r=subprocess.run(cmd,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=45)
        if r.returncode:raise ValueError('C/C++编译失败：\n'+r.stderr[-6000:])
        entry=output;manifest.update(compiler=Path(compiler).name,compiler_output=r.stderr)
    (root/'import_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    return UserController(kind,entry,manifest)
