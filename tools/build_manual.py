"""Rebuild the self-contained Chinese HTML tutorial (development dependency: mistune)."""
from pathlib import Path
import html,re
import mistune
ROOT=Path(__file__).resolve().parents[1]
md=(ROOT/'docs/中文使用教程.md').read_text(encoding='utf-8')
body=mistune.create_markdown(plugins=['table'])(md).replace('src="preview.png"','src="docs/preview.png"')
navigation=[]
def heading(match):
    label=html.unescape(re.sub('<[^>]+>','',match.group(1)))
    ident='section-'+str(len(navigation)+1);navigation.append((ident,label))
    return '<h2 id="'+ident+'">'+match.group(1)+'</h2>'
body=re.sub(r'<h2>(.*?)</h2>',heading,body)
nav=''.join('<a href="#'+ident+'">'+html.escape(label)+'</a>' for ident,label in navigation)
page='''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>智能车仿真完整中文使用教程</title><style>
:root{font-family:"Microsoft YaHei",system-ui,sans-serif;color:#243b47;background:#edf2f4;line-height:1.85}*{box-sizing:border-box}body{margin:0}nav{position:fixed;top:0;left:0;bottom:0;width:270px;overflow:auto;padding:24px;background:#10454e;color:white}nav strong{display:block;font-size:20px;margin-bottom:16px}nav a{display:block;color:#d5edef;text-decoration:none;padding:7px 0;font-size:13px;line-height:1.6}nav a:hover{color:white}article{max-width:1080px;margin:0 auto 0 290px;padding:42px;background:white}h1{font-size:30px;color:#10454e}h2{margin-top:48px;padding-top:18px;border-top:1px solid #d5e2e6;color:#10454e;scroll-margin-top:20px}h3{color:#2c6670}p,li{font-size:15px}pre{overflow:auto;background:#142f38;color:#ecf7f7;padding:18px;border-radius:6px;line-height:1.6}code{font-family:Consolas,monospace;font-size:13px}p code,li code,td code{background:#edf3f5;padding:2px 5px;border-radius:3px}table{width:100%;border-collapse:collapse;margin:22px 0;font-size:14px}th{background:#e7f1f2;text-align:left}td,th{border:1px solid #cbdde1;padding:10px}img{max-width:100%;height:auto;border:1px solid #d7e4e8}a{color:#116b7c}.tip{font-size:13px;color:#657d87}footer{margin-top:45px;color:#6a818a;font-size:12px}@media(max-width:900px){nav{position:static;width:auto;max-height:260px}article{margin:0;padding:24px}}@media print{nav{display:none}article{margin:0;padding:0;max-width:none}pre{white-space:pre-wrap}h2{break-before:auto}table,pre{break-inside:avoid}}
</style></head><body><nav><strong>GateCar Simulator</strong><span>完整中文教程</span>'''+nav+'''</nav><article><p class="tip">离线可读，无需启动仿真。按Ctrl+F查找关键词，点击左侧目录跳转。</p>'''+body+'''<footer>本地教程与下载包一起提供。接口以sdk/gatecar_api.h和docs/CODE_IMPORT.md为准。</footer></article></body></html>'''
(ROOT/'使用教程.html').write_text(page,encoding='utf-8')
assert len(navigation)==20,len(navigation)
print('MANUAL_BUILT',len(navigation),'chapters',len(page))
