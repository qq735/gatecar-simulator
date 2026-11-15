"""Optional portable reference controller. Never reads files outside this package."""
from pathlib import Path
import ctypes, hashlib, json, os, shutil, subprocess, sys, time

ROOT = Path(__file__).resolve().parent
INPUT_FIELDS = ['adc_l','adc_r','wheel_l','wheel_r','gyro_z','visual_error','visual_element','frame_new','line_valid','gap_hint','arm','kill','finish_mark']
OUTPUT_FIELDS = ['steer','left_pwm','right_pwm','fan_command','target_l','target_r','mag_error','mag_valid','enabled','element','gate','fault','finish_count','finished']

def find_compiler():
    explicit = os.environ.get('CC')
    if explicit:
        candidate = shutil.which(explicit) or (explicit if Path(explicit).is_file() else None)
        if candidate: return candidate
    for name in ['gcc', 'clang', 'cc']:
        found = shutil.which(name)
        if found: return found
    if sys.platform == 'win32':
        for prefix in ['C:/msys64/ucrt64/bin', 'C:/msys64/mingw64/bin', 'C:/msys2/ucrt64/bin']:
            p = Path(prefix)/'gcc.exe'
            if p.is_file(): return str(p)
    return None

def shared_compile_command(compiler, sources, include_dirs, output, cplusplus=False):
    if cplusplus:
        candidate=Path(compiler).with_name('clang++'+Path(compiler).suffix if 'clang' in Path(compiler).name else 'g++'+Path(compiler).suffix)
        if candidate.is_file():compiler=str(candidate)
        else:
            compiler=shutil.which('g++') or shutil.which('clang++')
            if not compiler:raise RuntimeError('C++项目需要g++或clang++编译器')
    cmd = [compiler, '-O2', '-Wall', '-Wextra', '-std=c++17' if cplusplus else '-std=c11']
    cmd += ['-dynamiclib'] if sys.platform == 'darwin' else ['-shared']
    if sys.platform != 'win32': cmd += ['-fPIC']
    elif 'clang' not in Path(compiler).name: cmd += ['-static-libgcc']+(['-static-libstdc++'] if cplusplus else [])
    for directory in include_dirs: cmd += ['-I', str(directory)]
    return cmd + [str(p) for p in sources] + ['-o', str(output), '-lm']

class NativeCore:
    def __init__(self):
        self.library = None; self.handle = None; self.config = None
        self.manifest = dict(status='not_compiled', source_directory='control_src', compiler_available=bool(find_compiler()))

    def fork(self):
        other = NativeCore()
        other.library = self.library; other.manifest = dict(self.manifest)
        if self.library is not None:
            other.handle = self.library.sim_create()
            if not other.handle: raise RuntimeError('Cannot allocate reference controller')
        return other

    def rebuild(self):
        compiler = find_compiler()
        if not compiler: raise RuntimeError('找不到C编译器。安装GCC/Clang并加入PATH，或先导入Python控制器。')
        src = ROOT/'control_src'; paths = [src/n for n in ['CarCore.c','CarProfiles.c','CarCore.h']]
        if not all(p.is_file() for p in paths): raise RuntimeError('control_src缺少CarCore.c、CarProfiles.c或CarCore.h')
        build = ROOT/'build'; build.mkdir(exist_ok=True)
        output = build/('reference_'+str(time.time_ns())+('.dll' if os.name=='nt' else '.dylib' if sys.platform=='darwin' else '.so'))
        cmd = shared_compile_command(compiler,[ROOT/'core_bridge.c',*paths[:2]],[src],output)
        cmd.insert(1,'-DCAR_HOST')
        r = subprocess.run(cmd,capture_output=True,text=True,encoding='utf-8',errors='replace',timeout=60)
        if r.returncode: raise RuntimeError('参考C核心编译失败：\n'+r.stderr[-6000:])
        lib=ctypes.CDLL(str(output)); lib.sim_create.restype=ctypes.c_void_p
        lib.sim_destroy.argtypes=[ctypes.c_void_p]
        lib.sim_reset.argtypes=[ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32)]
        lib.sim_tick.argtypes=[ctypes.c_void_p,ctypes.POINTER(ctypes.c_int32),ctypes.POINTER(ctypes.c_int32)]
        handle=lib.sim_create()
        if not handle: raise RuntimeError('参考C核心内存分配失败')
        self.close(); self.library=lib; self.handle=handle
        self.manifest=dict(status='compiled',source_directory='control_src',compiler=Path(compiler).name,
                           sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},compiler_output=r.stderr)
        (build/'compiled_core.json').write_text(json.dumps(self.manifest,ensure_ascii=False,indent=2),encoding='utf-8')
        if self.config: self.reset(self.config)
        return self.manifest

    def reset(self,config):
        self.config=dict(config)
        if config['controller']=='native_c' and self.library is None: self.rebuild()
        if self.library is not None:
            v=[1,0,int(config['mag_min_sum']),int(config['mag_sign']),int(config['adc_zero']),int(config['adc_zero']),int(config['gate_timeout_ms']/5),500,1000]
            self.library.sim_reset(self.handle,(ctypes.c_int32*len(v))(*v))

    def step(self,data):
        if self.library is None: self.rebuild()
        values=[]
        for name in INPUT_FIELDS:
            v=int(round(data.get(name,0)))
            values.append(max(0,min(4095,v)) if name.startswith('adc_') else max(-32767,min(32767,v)))
        out=(ctypes.c_int32*len(OUTPUT_FIELDS))()
        self.library.sim_tick(self.handle,(ctypes.c_int32*len(values))(*values),out)
        return dict(zip(OUTPUT_FIELDS,list(out)))

    def close(self):
        if self.library is not None and self.handle:
            self.library.sim_destroy(self.handle); self.handle=None
