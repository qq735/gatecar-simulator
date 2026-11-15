"""Create an isolated dependency environment when needed, then start the local service."""
from pathlib import Path
import importlib.util,os,subprocess,sys,venv
ROOT=Path(__file__).resolve().parent
def main():
    for stream in [sys.stdout,sys.stderr]:
        if hasattr(stream,'reconfigure'):stream.reconfigure(encoding='utf-8')
    if sys.version_info<(3,10):raise SystemExit('Please install Python 3.10 or newer (64-bit).')
    env_python=ROOT/'.venv'/('Scripts/python.exe' if os.name=='nt' else 'bin/python')
    in_env=Path(sys.prefix).resolve()==(ROOT/'.venv').resolve()
    if env_python.is_file() and not in_env:return subprocess.call([str(env_python),str(ROOT/'start.py'),*sys.argv[1:]])
    missing=[x for x in ['numpy','scipy','shapely'] if importlib.util.find_spec(x) is None]
    if missing:
        if not in_env:
            print('Creating .venv for simulator dependencies...',flush=True)
            venv.EnvBuilder(with_pip=True).create(ROOT/'.venv')
            subprocess.check_call([str(env_python),'-m','pip','install','-r',str(ROOT/'requirements.txt')])
            return subprocess.call([str(env_python),str(ROOT/'start.py'),*sys.argv[1:]])
        subprocess.check_call([sys.executable,'-m','pip','install','-r',str(ROOT/'requirements.txt')])
    from server import main as serve
    serve()
    return 0
if __name__=='__main__':
    try:sys.exit(main())
    except Exception as exc:
        print('Startup failed:',exc,file=sys.stderr)
        print('Manual setup: python -m pip install -r requirements.txt',file=sys.stderr)
        sys.exit(1)
