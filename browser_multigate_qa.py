"""Actual WebGL/DOM verification through isolated headless Edge CDP."""
from pathlib import Path
import subprocess,tempfile,time,json,urllib.request,base64,websocket,os,shutil
ROOT=Path(__file__).parent.resolve();url=json.loads((ROOT/'build/server_status.json').read_text())['url']
edge=Path(os.environ.get('GATECAR_BROWSER') or shutil.which('msedge') or shutil.which('google-chrome') or shutil.which('chromium') or r'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe')
if not edge.is_file():raise RuntimeError('找不到Edge/Chrome，请设置GATECAR_BROWSER为可执行文件路径')
profile=tempfile.mkdtemp(prefix='gatecar_multigate_qa_');port=9488
proc=subprocess.Popen([str(edge),'--headless=new','--use-angle=swiftshader','--enable-unsafe-swiftshader',f'--remote-debugging-port={port}',
 '--remote-allow-origins=*',f'--user-data-dir={profile}','--no-first-run','--disable-background-networking','--window-size=1600,1000','about:blank'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
seq=0;errors=[]
try:
    deadline=time.monotonic()+25
    while time.monotonic()<deadline:
        try:pages=json.load(urllib.request.urlopen(f'http://127.0.0.1:{port}/json'));break
        except Exception:time.sleep(.2)
    page=next(p for p in pages if p['type']=='page')
    ws=websocket.create_connection(page['webSocketDebuggerUrl'],timeout=60)
    def cdp(method,params=None):
        global seq
        seq+=1;request=seq;ws.send(json.dumps(dict(id=request,method=method,params=params or {})))
        while True:
            response=json.loads(ws.recv())
            if response.get('method')=='Runtime.exceptionThrown':errors.append(response['params'])
            if response.get('method')=='Runtime.consoleAPICalled' and response.get('params',{}).get('type')=='error':errors.append(response['params'])
            if response.get('id')==request:
                if 'error' in response:raise RuntimeError(response['error'])
                return response.get('result',{})
    def js(expression,await_=False):
        r=cdp('Runtime.evaluate',dict(expression=expression,returnByValue=True,awaitPromise=await_))
        if 'exceptionDetails' in r:raise RuntimeError(r['exceptionDetails'])
        return r.get('result',{}).get('value')
    def shot(name):
        r=cdp('Page.captureScreenshot',dict(format='png',captureBeyondViewport=False));(ROOT/'build'/name).write_bytes(base64.b64decode(r['data']))
    cdp('Page.enable');cdp('Runtime.enable');cdp('Emulation.setDeviceMetricsOverride',dict(width=1600,height=1080,deviceScaleFactor=1,mobile=False))
    cdp('Page.navigate',dict(url=url+'/?qa=1'))
    deadline=time.monotonic()+30
    while time.monotonic()<deadline:
        if js('Boolean(window.__simTest?.ready&&window.__simTest?.modelLoaded)'):break
        time.sleep(.2)
    shot('multigate_boot.png')
    assert js('Boolean(window.__simTest?.ready&&window.__simTest?.modelLoaded)'),errors
    assert js('__simTest.state.config.course==="room" && __simTest.state.gate_count===11')
    js("document.querySelector('[data-view=top]').click()");time.sleep(.4);shot('multigate_top.png')
    from PIL import Image
    import numpy as np
    pixels=np.array(Image.open(ROOT/'build/multigate_top.png').convert('RGB'))
    assert np.all(pixels==[55,67,115],axis=2).sum()>10000,'Sampled blue must render unchanged'
    js("document.querySelector('#mapStyle').value='bw';document.querySelector('#mapStyle').dispatchEvent(new Event('change'))")
    time.sleep(.3);shot('multigate_blackwhite_toggle.png')
    js("document.querySelector('#mapStyle').value='bluewhite';document.querySelector('#mapStyle').dispatchEvent(new Event('change'))")
    assert js('__simTest.state.t===0')
    # DOM scene selection, physical speed control and actual user-facing counters.
    js("document.querySelector('#gate_mode').value='left_ring';document.querySelector('#gate_mode').dispatchEvent(new Event('change'))")
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        if js('__simTest.state.config.gate_mode==="left_ring" && __simTest.state.gate_count===1'):break
        time.sleep(.1)
    assert js('__simTest.state.gate_count===1')
    local=js("(async()=>{await __simTest.reset({course:'room',gate_mode:'left_ring',start_mode:'gate',controller:'route_demo'});await __simTest.fastRun(3000);return {status:__simTest.state.status,gates:__simTest.state.passed_gates,local:__simTest.state.local_gate_test};})()",True)
    assert local==dict(status='passed',gates=1,local=True),local
    shot('multigate_left_ring_pass.png')
    full=js("(async()=>{await __simTest.reset({course:'room',controller:'route_demo',gate_mode:'all',speed_scale:1.6});await __simTest.fastRun(12000);return {status:__simTest.state.status,time:__simTest.state.t,maxSpeed:__simTest.state.max_speed,gates:__simTest.state.passed_gates,lap:__simTest.state.lap_progress};})()",True)
    assert full['status']=='passed' and full['gates']==11 and full['maxSpeed']>2.1,full
    assert js("document.querySelector('#gateCounter').textContent==='11 / 11'")
    shot('multigate_full_lap_pass.png')
    # Trace count and bounded UI calls are independent of the wall-clock render multiplier.
    assert js("document.querySelector('#timeScale').value==='1'")
    # Import through the same file picker/FileReader used by a downloaded copy.
    imported=[]
    for filename in ['controller.py','controller.c']:
        if filename.endswith('.c') and not js('__simTest.info().core.compiler_available'):continue
        document=cdp('DOM.getDocument');node=cdp('DOM.querySelector',dict(nodeId=document['root']['nodeId'],selector='#controllerFile'))['nodeId']
        cdp('DOM.setFileInputFiles',dict(nodeId=node,files=[str(ROOT/'examples'/filename)]))
        deadline=time.monotonic()+15
        language='python' if filename.endswith('.py') else 'c'
        while time.monotonic()<deadline:
            if js(f'__simTest.state.user_controller?.language==="{language}"'):break
            time.sleep(.1)
        assert js(f'__simTest.state.config.controller==="user_code" && __simTest.state.user_controller?.language==="{language}"'),js("document.querySelector('#toast').textContent")
        js("(async()=>{await __simTest.advance(100);})()",True)
        assert js('__simTest.state.speed>0 && __simTest.state.fault===0')
        imported.append(language)
    shot('user_code_import.png')
    js("document.querySelector('#removeController').click()")
    deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        if js('__simTest.state.user_controller===null'):break
        time.sleep(.1)
    assert js('__simTest.state.user_controller===null && __simTest.state.config.controller==="route_demo"')
    # Full tutorial remains available both offline and through the local server.
    manual=js("fetch('/manual.html').then(r=>r.text()).then(t=>({ok:t.includes('20. 仿真结果的边界'),chapters:(t.match(/<h2 id=/g)||[]).length}))",True)
    assert manual['ok'] and manual['chapters']==20,manual

    cdp('Emulation.setDeviceMetricsOverride',dict(width=1024,height=800,deviceScaleFactor=1,mobile=False));time.sleep(.2)
    assert js('document.documentElement.scrollWidth<=innerWidth+2')
    shot('multigate_1024.png')
    cdp('Emulation.setDeviceMetricsOverride',dict(width=1440,height=1000,deviceScaleFactor=1,mobile=False))
    cdp('Page.navigate',dict(url=(ROOT/'使用教程.html').as_uri()))
    deadline=time.monotonic()+8
    while time.monotonic()<deadline:
        if js('document.querySelectorAll("h2").length===20'):break
        time.sleep(.1)
    assert js('document.querySelectorAll("h2").length===20 && document.images[0].complete && document.images[0].naturalWidth>0')
    assert js('document.documentElement.scrollWidth<=innerWidth+2')
    shot('chinese_manual_offline.png')
    assert not errors,errors
    report=dict(runtime_errors=errors,model_loaded=True,initial_gate_count=11,local_ring=local,full_lap=full,imported_code_languages=imported,chinese_manual_chapters=20,offline_manual_verified=True,playback_multiplier=1,viewport_1024_no_horizontal_overflow=True)
    (ROOT/'build/browser_multigate_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print('BROWSER_QA_PASS',json.dumps(report,ensure_ascii=False),flush=True)
    cdp('Browser.close')
finally:
    if proc.poll() is None:proc.terminate()
