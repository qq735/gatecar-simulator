"""Local simulator HTTP service and independent per-tab code import."""
from http.server import ThreadingHTTPServer,SimpleHTTPRequestHandler
from pathlib import Path
import argparse,base64,binascii,io,json,re,threading,time,urllib.parse,webbrowser,zipfile
from simulation import Simulation,DEFAULTS
from user_controller import MAX_UPLOAD

ROOT=Path(__file__).resolve().parent;WEB=ROOT/'web'
class Handler(SimpleHTTPRequestHandler):
    def __init__(self,*args,**kwargs):super().__init__(*args,directory=str(WEB),**kwargs)
    def log_message(self,fmt,*args):
        if len(args)>1 and str(args[1]) not in ['200','304']:super().log_message(fmt,*args)
    def session(self):
        token=self.headers.get('X-Sim-Session') or urllib.parse.parse_qs(urllib.parse.urlsplit(self.path).query).get('session',[''])[0]
        if not token:return self.server.default_sim
        if not re.fullmatch(r'[A-Za-z0-9_-]{8,80}',token):raise ValueError('Invalid simulation session')
        sessions=self.server.sessions
        if token not in sessions:
            if len(sessions)>=16:
                expired=min(sessions,key=lambda k:sessions[k][1]);old,_=sessions.pop(expired);old.close()
            sessions[token]=[Simulation(self.server.default_sim.core.fork()),time.time()]
        sessions[token][1]=time.time();return sessions[token][0]
    def json(self,value,status=200):
        data=json.dumps(value,ensure_ascii=False,allow_nan=False).encode('utf-8');self.send_response(status)
        self.send_header('Content-Type','application/json; charset=utf-8');self.send_header('Cache-Control','no-store')
        self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
    def do_GET(self):
        try:
            path=urllib.parse.urlsplit(self.path).path
            if path in ['/manual.html','/docs/preview.png']:
                p=ROOT/'使用教程.html' if path=='/manual.html' else ROOT/'docs/preview.png'
                data=p.read_bytes();self.send_response(200)
                self.send_header('Content-Type','text/html; charset=utf-8' if path=='/manual.html' else 'image/png')
                self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data);return
            template_paths={'/templates/controller.py':ROOT/'examples/controller.py','/templates/controller.c':ROOT/'examples/controller.c',
                            '/templates/gatecar_api.h':ROOT/'sdk/gatecar_api.h','/templates/code-import.md':ROOT/'docs/CODE_IMPORT.md'}
            if path in template_paths:
                p=template_paths[path];data=p.read_bytes();self.send_response(200);self.send_header('Content-Type','text/plain; charset=utf-8')
                self.send_header('Content-Disposition','attachment; filename="'+p.name+'"');self.send_header('Content-Length',str(len(data)))
                self.end_headers();self.wfile.write(data);return
            if path=='/templates/c-project.zip':
                b=io.BytesIO()
                with zipfile.ZipFile(b,'w',zipfile.ZIP_DEFLATED) as z:
                    z.write(ROOT/'examples/controller.c','controller.c');z.write(ROOT/'sdk/gatecar_api.h','gatecar_api.h')
                data=b.getvalue();self.send_response(200);self.send_header('Content-Type','application/zip');self.send_header('Content-Disposition','attachment; filename="controller-example.zip"')
                self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data);return
            if not path.startswith('/api/'):return super().do_GET()
            with self.server.lock:
                sim=self.session()
                if path=='/api/state':return self.json(sim.snapshot())
                if path=='/api/info':return self.json(dict(defaults=DEFAULTS,core=sim.core.manifest,dt_ms=5,
                    model=json.loads((WEB/'assets/model_manifest.json').read_text(encoding='utf-8')),code_import=dict(formats=['.py','.c','.cpp','.zip'],api_version=1,max_bytes=MAX_UPLOAD)))
                if path=='/api/field':return self.json(sim.field_grid())
                if path=='/api/course':return self.json(sim.course.payload())
                if path=='/api/log.csv':
                    data=('\ufeff'+sim.csv()).encode('utf-8');self.send_response(200);self.send_header('Content-Type','text/csv; charset=utf-8')
                    self.send_header('Content-Disposition','attachment; filename="simulation_log.csv"');self.send_header('Content-Length',str(len(data)))
                    self.end_headers();self.wfile.write(data);return
            return self.json(dict(error='Unknown endpoint'),404)
        except (ValueError,TypeError) as e:return self.json(dict(error=str(e)),400)
        except Exception as e:return self.json(dict(error=str(e)),500)
    def do_POST(self):
        try:
            origin=self.headers.get('Origin');host=self.headers.get('Host','')
            if origin and origin not in [f'http://127.0.0.1:{self.server.server_port}',f'http://localhost:{self.server.server_port}',f'http://{host}']:
                return self.json(dict(error='不接受外部来源请求'),403)
            if self.headers.get('Sec-Fetch-Site')=='cross-site':return self.json(dict(error='不接受跨站请求'),403)
            size=int(self.headers.get('Content-Length','0'))
            if size<0 or size>3_000_000:return self.json(dict(error='请求超过3 MB'),413)
            data=json.loads(self.rfile.read(size) or b'{}')
            with self.server.lock:
                sim=self.session()
                if self.path=='/api/reset':return self.json(sim.reset(data))
                if self.path=='/api/step':return self.json(sim.step(data.get('ticks',10)))
                if self.path=='/api/run':
                    remaining=max(1,min(36000,int(data.get('ticks',18000))))
                    while remaining>0 and sim.status in ['ready','running']:
                        n=min(remaining,200);sim.step(n);remaining-=n
                    return self.json(dict(state=sim.snapshot(),samples=sim.history[-4800::4]))
                if self.path=='/api/kill':sim.cfg['kill_s']=sim.time;return self.json(sim.step(1))
                if self.path=='/api/rebuild':
                    manifest=sim.core.rebuild();sim.reset(sim.cfg);return self.json(dict(core=manifest,state=sim.snapshot()))
                if self.path=='/api/controller/import':
                    try:content=base64.b64decode(data.get('content_base64',''),validate=True)
                    except (binascii.Error,ValueError) as e:raise ValueError('代码文件编码错误') from e
                    return self.json(sim.import_controller(data.get('filename',''),content))
                if self.path=='/api/controller/remove':return self.json(sim.use_reference_controller())
            return self.json(dict(error='Unknown endpoint'),404)
        except (ValueError,TypeError,RuntimeError,zipfile.BadZipFile) as e:return self.json(dict(error=str(e)),400)
        except Exception as e:return self.json(dict(error=str(e)),500)

class SimulatorServer(ThreadingHTTPServer):
    daemon_threads=True
    def __init__(self,address):
        super().__init__(address,Handler);self.default_sim=Simulation();self.sessions={};self.lock=threading.RLock()
    def server_close(self):
        for sim,_ in self.sessions.values():sim.close()
        self.sessions.clear();self.default_sim.close();super().server_close()

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--port',type=int,default=8870);parser.add_argument('--no-browser',action='store_true')
    parser.add_argument('--host',default='127.0.0.1',choices=['127.0.0.1','0.0.0.0']);args=parser.parse_args();server=None
    for port in range(args.port,args.port+10):
        try:server=SimulatorServer((args.host,port));break
        except OSError:continue
    if server is None:raise RuntimeError('本地端口被占用，请用--port指定其他端口')
    url=f'http://127.0.0.1:{server.server_port}';build=ROOT/'build';build.mkdir(exist_ok=True)
    (build/'server_status.json').write_text(json.dumps(dict(url=url,port=server.server_port,started=time.time()),indent=2),encoding='utf-8')
    print('SIMULATOR_READY',url,flush=True)
    if not args.no_browser:threading.Timer(1,lambda:webbrowser.open(url+'/?view=top')).start()
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()
if __name__=='__main__':main()
