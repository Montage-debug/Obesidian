from pathlib import Path
from zipfile import ZipFile
from lxml import etree
from PIL import Image, ImageDraw
import urllib.request, json, csv
B=Path(__file__).parent; ROOT=B.parent.parent
for name,url in [('eye_hand.png','https://docs.opencv.org/4.10.0/hand-eye_figure.png'),('pinhole.png','https://docs.opencv.org/4.10.0/pinhole_camera_model.png'),('opencv_LICENSE.txt','https://raw.githubusercontent.com/opencv/opencv/4.10.0/LICENSE')]:
    try: urllib.request.urlretrieve(url,B/'assets'/name)
    except Exception as e: print(name,type(e).__name__)
with ZipFile(B/'visual_source.docx') as z:
    for n in z.namelist():
        if n.startswith('word/media/'):
            (B/'assets'/('visual_'+Path(n).name)).write_bytes(z.read(n))
with ZipFile(ROOT/'穴位识别算法/waypoint机器视觉/视觉汇报.pptx') as z:
    out=[]
    for n in sorted([n for n in z.namelist() if n.startswith('ppt/slides/slide') and n.endswith('.xml')],key=lambda n:int(Path(n).stem[5:])):
        x=etree.fromstring(z.read(n)); out.append(n+'\n'+'\n'.join(x.xpath('//*[local-name()="t"]/text()')))
    (B/'ppt_text.txt').write_text('\n\n'.join(out),encoding='utf-8')
ims=[]
for f in (B/'assets').glob('visual_*'):
    try:
        im=Image.open(f); im.thumbnail((460,250)); ims.append((f,im.copy().convert('RGB')))
    except Exception: pass
for start in range(0,len(ims),12):
    sheet=Image.new('RGB',(1440,310*((len(ims[start:start+12])+2)//3)),'white');d=ImageDraw.Draw(sheet)
    for k,(f,im) in enumerate(ims[start:start+12]):
        x=(k%3)*480; y=(k//3)*310;sheet.paste(im,(x+(480-im.width)//2,y));d.text((x+10,y+270),f.name,fill='black')
    sheet.save(B/f'inventory_visual_{start//12}.jpg')
print('Visual media',len(ims))
