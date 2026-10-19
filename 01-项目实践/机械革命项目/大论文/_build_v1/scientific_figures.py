from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
import csv,math
B=Path(__file__).parent;ROOT=B.parent.parent
def ft(n):return ImageFont.truetype('C:/Windows/Fonts/arial.ttf',n)
def cn(n):return ImageFont.truetype('C:/Windows/Fonts/simsun.ttc',n)
for folder,csvname,name in [('neck_x_first','颈模results.csv','neck'),('train_x_navel_only','脐模results.csv','navel'),('leg_two','膝模results.csv','knee')]:
    records=list(csv.DictReader((ROOT/'穴位识别算法/算力云实验结果'/folder/csvname).open(encoding='utf-8-sig')))
    records=[{k.strip():float(v) for k,v in r.items()} for r in records]
    im=Image.new('RGB',(1800,620),'white');d=ImageDraw.Draw(im)
    for idx,(key,title) in enumerate([('val/pose_loss','(a) Validation pose loss'),('metrics/mAP50-95(P)','(b) Pose mAP50-95')]):
        left=105+idx*890;right=835+idx*890;top=90;bottom=510
        vals=[r[key] for r in records];mx=1.0 if idx==1 else max(vals)*1.05;epochs=[r['epoch'] for r in records]
        d.text((left+120,20),title,font=ft(30),fill='black')
        for k in range(6):
            y=bottom-(bottom-top)*k/5;d.line((left,y,right,y),fill='#dddddd',width=1);lab=f'{mx*k/5:.2f}' if mx<10 else f'{mx*k/5:.1f}';d.text((left-90,y-15),lab,font=ft(24),fill='black')
        for k in range(6):
            x=left+(right-left)*k/5;d.line((x,top,x,bottom),fill='#eeeeee');d.text((x-15,bottom+16),str(round(max(epochs)*k/5)),font=ft(24),fill='black')
        pts=[(left+(right-left)*e/max(epochs),bottom-(bottom-top)*v/mx) for e,v in zip(epochs,vals)]
        d.line(pts,fill='#215f99',width=4);d.line((left,top,left,bottom,right,bottom),fill='black',width=2);d.text((left+300,565),'Epoch',font=ft(26),fill='black')
    im.save(B/'assets'/f'{name}_metrics.png')
im=Image.new('RGB',(1500,1000),'white');d=ImageDraw.Draw(im)
N=(740,140);H=(740,790);SL=(430,250);SR=(1050,250);HL=(560,790);HR=(920,790)
d.line((SL,SR),fill='#555555',width=4);d.line((HL,HR),fill='#555555',width=4);d.line((N,H),fill='#555555',width=4)
for pt,lab,off in [(N,'N 颈部锚点',(25,-35)),(H,'H 髋中点',(25,20)),(SL,'左肩',(-120,-45)),(SR,'右肩',(20,-45)),(HL,'左髋',(-110,15)),(HR,'右髋',(20,15))]:
    x,y=pt;d.ellipse((x-8,y-8,x+8,y+8),fill='#1c5c90');d.text((x+off[0],y+off[1]),lab,font=cn(31),fill='black')
def arrow(a,b):
    d.line((a,b),fill='black',width=4);ang=math.atan2(b[1]-a[1],b[0]-a[0]);p1=(b[0]-22*math.cos(ang-.5),b[1]-22*math.sin(ang-.5));p2=(b[0]-22*math.cos(ang+.5),b[1]-22*math.sin(ang+.5));d.polygon([b,p1,p2],fill='black')
arrow((740,400),(740,610));d.text((760,490),'纵向 e_l',font=cn(32),fill='black')
arrow((740,440),(1020,440));d.text((850,455),'横向 e_t',font=cn(32),fill='black')
d.text((90,400),'初始纵向尺度',font=cn(34),fill='black');d.text((90,455),'s_l = |H - N| / 22',font=ft(34),fill='black')
d.text((85,560),'横向尺度取肩、髋',font=cn(32),fill='black');d.text((85,610),'两种归一化距离的较小值',font=cn(32),fill='black')
d.text((210,910),'原理示意，点位不表示真实测量或穴位真值',font=cn(34),fill='black')
im.save(B/'assets/geometry.png')
print('4 scientific figures written from raw CSV and geometry definitions')
