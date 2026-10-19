from pathlib import Path
from zipfile import ZipFile
from lxml import etree
from docx import Document
from PIL import Image,ImageOps,ImageDraw,ImageFont
import json,hashlib,csv

ROOT=Path(r'E:\obsidian_知识库\01-项目实践\机械革命项目')
BUILD=ROOT/'大论文'/'_build_v1'
BUILD.mkdir(exist_ok=True)
(BUILD/'assets').mkdir(exist_ok=True)
REF=Path(r'E:\xwechat_files\wxid_6rawr3ohzsdb22_ac6d\msg\file\2026-10\MS基于NX MCD的装配机械臂轨迹规划与虚拟调试研究.docx')
SOURCES={
 'reference':REF,
 'scrrt':ROOT/'SC-RRT小论文编写/SC-RRT论文-增机械臂实验版/zh/修订交付_20260907/第九版_SC-RRT_中文完整修订稿_黑色加粗.docx',
 'scrrt8':ROOT/'SC-RRT小论文编写/SC-RRT论文-增机械臂实验版/zh/第八版-完整重构-SC-RRT增加实验版.docx',
 'bridge':Path(r'E:\xwechat_files\wxid_6rawr3ohzsdb22_ac6d\msg\file\2026-10\视觉目标到机械臂运动约束的衔接与验证.docx'),
 'outline':ROOT/'大论文/郭兴雨大论文目录.docx'
}
ns={'w':'http://schemas.openxmlformats.org/wordprocessingml/2006/main','a':'http://schemas.openxmlformats.org/drawingml/2006/main'}
evidence={}
for key,path in SOURCES.items():
 with ZipFile(path) as z:
  x=etree.fromstring(z.read('word/document.xml'))
  pars=[''.join(p.xpath('.//w:t/text()',namespaces=ns)) for p in x.xpath('//w:body//w:p',namespaces=ns)]
  (BUILD/(key+'_text.txt')).write_text('\n'.join(f'[{i}] {t}' for i,t in enumerate(pars)),encoding='utf-8')
  imgs=[]
  if key in ['scrrt','scrrt8','bridge']:
   for n in z.namelist():
    if n.startswith('word/media/'):
     target=BUILD/'assets'/f'{key}_{Path(n).name}'
     target.write_bytes(z.read(n))
     try:
      im=Image.open(target); imgs.append({'path':str(target),'size':im.size})
     except: pass
  evidence[key]={'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'images':imgs}
doc=Document(REF)
evidence['layout']={'sections':[], 'styles':[]}
for s in doc.sections:
 evidence['layout']['sections'].append({k:round(getattr(s,k).mm,2) if getattr(s,k) else None for k in ['page_width','page_height','top_margin','bottom_margin','left_margin','right_margin','header_distance','footer_distance']})
for s in doc.styles:
 if s.type==1 and (s.style_id in ['Normal','1','2','3','af1'] or 'heading' in s.name.lower() or 'toc' in s.name.lower()):
  evidence['layout']['styles'].append({'id':s.style_id,'name':s.name,'xml':s.element.xml})
(BUILD/'source_manifest.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')
# QA-only image inventory contact sheets; originals remain untouched.
for key in ['scrrt','scrrt8','bridge']:
 imgs=evidence[key]['images']; tiles=[]
 for item in imgs:
  im=Image.open(item['path']).convert('RGB'); im.thumbnail((460,270))
  tile=Image.new('RGB',(480,310),'white'); tile.paste(im,((480-im.width)//2,22))
  ImageDraw.Draw(tile).text((8,288),Path(item['path']).name,fill='black');tiles.append(tile)
 for batch in range(0,len(tiles),12):
  subset=tiles[batch:batch+12]; sheet=Image.new('RGB',(480*3,310*((len(subset)+2)//3)),'#dddddd')
  for j,t in enumerate(subset):sheet.paste(t,((j%3)*480,(j//3)*310))
  sheet.save(BUILD/f'inventory_{key}_{batch//12}.jpg')
print(json.dumps({'layout':evidence['layout']['sections'],'media':{k:len(evidence[k]['images']) for k in ['scrrt','scrrt8','bridge']}},ensure_ascii=False))
