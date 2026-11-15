"""Build a source-only, reproducible release archive from an explicit allowlist."""
from pathlib import Path
import hashlib,json,zipfile
ROOT=Path(__file__).resolve().parents[1]
def package(output=None):
    dest=Path(output) if output else ROOT/'dist/gatecar-simulator.zip';dest.parent.mkdir(parents=True,exist_ok=True)
    roots=['control_src','sdk','examples','web','docs','tests','tools','.github']
    files={p for name in roots for p in (ROOT/name).rglob('*') if p.is_file()}
    files.update(p for p in ROOT.iterdir() if p.is_file() and (p.suffix in ['.py','.md','.txt','.json','.cmd','.sh','.html'] or p.name in ['LICENSE','.gitignore','.gitattributes']))
    # core_bridge is required by the optional reference C controller.
    files.add(ROOT/'core_bridge.c')
    files={p for p in files if '__pycache__' not in p.parts and p.suffix not in ['.pyc','.dll','.so','.dylib']}
    with zipfile.ZipFile(dest,'w',zipfile.ZIP_DEFLATED,compresslevel=9) as z:
        for p in sorted(files):
            info=zipfile.ZipInfo('gatecar-simulator/'+p.relative_to(ROOT).as_posix(),date_time=(2026,11,15,0,0,0))
            info.compress_type=zipfile.ZIP_DEFLATED;info.external_attr=(0o755 if p.name=='start.sh' else 0o644)<<16
            z.writestr(info,p.read_bytes())
    with zipfile.ZipFile(dest) as z:assert z.testzip() is None
    result=dict(file=dest.name,bytes=dest.stat().st_size,files=len(files),sha256=hashlib.sha256(dest.read_bytes()).hexdigest())
    dest.with_suffix('.sha256').write_text(result['sha256']+'  '+dest.name+'\n',encoding='ascii')
    print(json.dumps(result),flush=True);return dest
if __name__=='__main__':package()
