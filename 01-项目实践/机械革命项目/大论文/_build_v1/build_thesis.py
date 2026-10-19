from pathlib import Path
from copy import deepcopy
import re,json,csv
from docx import Document
from docx.shared import Mm,Pt,RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH,WD_BREAK
from docx.enum.section import WD_SECTION_START
from docx.enum.table import WD_TABLE_ALIGNMENT,WD_CELL_VERTICAL_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from PIL import Image

B=Path(__file__).parent; ROOT=B.parent.parent
M=json.loads((B/'source_manifest.json').read_text(encoding='utf-8'))
doc=Document(M['reference']['path'])
body=doc._element.body
for e in list(body): body.remove(e)
sec=OxmlElement('w:sectPr');body.append(sec)
missing=[]; figures=[]; equations=[]; tables=[]; section='封面'

def font(st,size,east='宋体',bold=False):
    st.font.name='Times New Roman';st.font.size=Pt(size);st.font.bold=bold;st.font.color.rgb=RGBColor(0,0,0)
    pr=st.element.get_or_add_rPr();rf=pr.find(qn('w:rFonts'))
    if rf is None:rf=OxmlElement('w:rFonts');pr.insert(0,rf)
    for a in ['asciiTheme','eastAsiaTheme','hAnsiTheme','cstheme']:
        rf.attrib.pop(qn('w:'+a),None)
    rf.set(qn('w:eastAsia'),east);rf.set(qn('w:ascii'),'Times New Roman');rf.set(qn('w:hAnsi'),'Times New Roman')
    co=pr.find(qn('w:color'))
    if co is not None:
        for a in ['themeColor','themeShade','themeTint']:co.attrib.pop(qn('w:'+a),None)
    pp=st.element.find(qn('w:pPr'))
    if pp is not None:
        for name in ['numPr','pBdr','shd','tabs','ind']:
            for el in list(pp.findall(qn('w:'+name))):pp.remove(el)
for name in ['Normal','Title','Subtitle','Heading 1','Heading 2','Heading 3','Caption','TOC Heading','Header','Footer']:
    if name not in doc.styles:doc.styles.add_style(name,1)
    font(doc.styles[name],12)
n=doc.styles['Normal'].paragraph_format;n.first_line_indent=Pt(24);n.line_spacing=1.5;n.space_after=Pt(0);n.space_before=Pt(0);n.widow_control=True
for i,size in [(1,16),(2,14),(3,12)]:
    st=doc.styles[f'Heading {i}'];font(st,size,'黑体',False);pf=st.paragraph_format
    pf.first_line_indent=Pt(0);pf.left_indent=Pt(0);pf.keep_with_next=True;pf.space_before=Pt(12 if i<3 else 8);pf.space_after=Pt(8 if i<3 else 5);pf.line_spacing=1.2
    pf.page_break_before=(i==1);pf.alignment=WD_ALIGN_PARAGRAPH.CENTER if i==1 else WD_ALIGN_PARAGRAPH.LEFT
font(doc.styles['Title'],22,'黑体');font(doc.styles['Caption'],10.5)
for nm in ['Title','Subtitle','TOC Heading']:
    pp=doc.styles[nm].element.get_or_add_pPr()
    for e in list(pp.findall(qn('w:outlineLvl'))):pp.remove(e)
    e=OxmlElement('w:outlineLvl');e.set(qn('w:val'),'9');pp.append(e)
cap=doc.styles['Caption'].paragraph_format;cap.first_line_indent=Pt(0);cap.alignment=WD_ALIGN_PARAGRAPH.CENTER;cap.line_spacing=1.15;cap.space_after=Pt(6);cap.space_before=Pt(4)
for name in ['TOC 1','TOC 2','TOC 3']:
    if name not in doc.styles:doc.styles.add_style(name,1)
    font(doc.styles[name],10.5);pf=doc.styles[name].paragraph_format;pf.line_spacing=1.15;pf.space_after=Pt(2);pf.first_line_indent=Pt(0);pf.left_indent=Pt(12*(int(name[-1])-1))

def setting(s):
    s.page_width=Mm(210);s.page_height=Mm(297);s.top_margin=Mm(28);s.bottom_margin=Mm(20);s.left_margin=Mm(25);s.right_margin=Mm(25);s.header_distance=Mm(17);s.footer_distance=Mm(12)
    for a in ['w:titlePg','w:pgNumType','w:docGrid']:
        for e in list(s._sectPr.findall(qn(a))):s._sectPr.remove(e)
setting(doc.sections[0])
# Disable inherited document grids, mirrored headers and tracked-change residue.
for name in ['evenAndOddHeaders','trackRevisions','mirrorMargins']:
    for e in list(doc.settings.element.findall(qn('w:'+name))):doc.settings.element.remove(e)
def field(p,code,cache=''):
    r=p.add_run();f=OxmlElement('w:fldChar');f.set(qn('w:fldCharType'),'begin');r._r.append(f)
    r=p.add_run();it=OxmlElement('w:instrText');it.set(qn('xml:space'),'preserve');it.text=' '+code+' ';r._r.append(it)
    r=p.add_run();f=OxmlElement('w:fldChar');f.set(qn('w:fldCharType'),'separate');r._r.append(f)
    p.add_run(cache);r=p.add_run();f=OxmlElement('w:fldChar');f.set(qn('w:fldCharType'),'end');r._r.append(f)
def pagesection(numberfmt='decimal',start=1):
    s=doc.add_section(WD_SECTION_START.NEW_PAGE);setting(s)
    s.header.is_linked_to_previous=False;s.footer.is_linked_to_previous=False
    h=s.header.paragraphs[0];h.text='上海应用技术大学  硕士学位论文';h.alignment=WD_ALIGN_PARAGRAPH.CENTER;h.paragraph_format.first_line_indent=Pt(0);font(doc.styles['Header'],9)
    f=s.footer.paragraphs[0];f.alignment=WD_ALIGN_PARAGRAPH.CENTER;f.paragraph_format.first_line_indent=Pt(0);field(f,'PAGE','1');font(doc.styles['Footer'],9)
    el=OxmlElement('w:pgNumType');el.set(qn('w:fmt'),numberfmt);el.set(qn('w:start'),str(start));s._sectPr.append(el)
    return s
def p(text='',style=None,center=False):
    x=doc.add_paragraph(text,style);x.paragraph_format.widow_control=True
    if center:x.alignment=WD_ALIGN_PARAGRAPH.CENTER;x.paragraph_format.first_line_indent=Pt(0)
    return x
def gap(code,text,kind='待补'):
    if any(v['id']==code for v in missing):raise ValueError('duplicate gap '+code)
    x=p();r=x.add_run(f'【{kind}{code}：{text}】');r.font.color.rgb=RGBColor.from_string('C00000');r.font.size=Pt(11)
    missing.append({'id':code,'section':section,'kind':kind,'need':text});return x
def heading(t,level):
    global section
    section=t
    if t=='目录':
        doc.styles['TOC Heading'].base_style=doc.styles['Normal']
        x=doc.add_paragraph(t,'TOC Heading')
        x.paragraph_format.alignment=WD_ALIGN_PARAGRAPH.CENTER
        x.paragraph_format.page_break_before=True
        x.paragraph_format.keep_with_next=True
        x.paragraph_format.first_line_indent=Pt(0)
        for r in x.runs:r.font.size=Pt(16);r.font.name='黑体'
    else:x=doc.add_heading(t,level)
    for e in list(x._p.get_or_add_pPr().findall(qn('w:numPr'))):x._p.get_or_add_pPr().remove(e)
    return x
def table(title,headers,rows,widths=None,red=False):
    tables.append(title);c=p(title,'Caption');c.paragraph_format.keep_with_next=True
    t=doc.add_table(rows=1,cols=len(headers));t.alignment=WD_TABLE_ALIGNMENT.CENTER;t.autofit=False
    if widths is None:widths=[160/len(headers)]*len(headers)
    for col,w in zip(t.columns,widths):col.width=Mm(w)
    pr=t._tbl.tblPr
    bord=OxmlElement('w:tblBorders')
    for nm in ['top','left','bottom','right','insideH','insideV']:
        z=OxmlElement('w:'+nm);z.set(qn('w:val'),'single');z.set(qn('w:sz'),'4');z.set(qn('w:color'),'D9D9D9');bord.append(z)
    pr.append(bord);mar=OxmlElement('w:tblCellMar')
    for nm,val in [('top',65),('bottom',65),('left',75),('right',75)]:
        z=OxmlElement('w:'+nm);z.set(qn('w:w'),str(val));z.set(qn('w:type'),'dxa');mar.append(z)
    pr.append(mar)
    for k,values in enumerate([headers]+rows):
        cells=t.rows[0].cells if k==0 else t.add_row().cells
        for j,(c,txt) in enumerate(zip(cells,values)):
            c.width=Mm(widths[j]);c.vertical_alignment=WD_CELL_VERTICAL_ALIGNMENT.CENTER
            pp=c.paragraphs[0];pp.paragraph_format.first_line_indent=Pt(0);pp.paragraph_format.line_spacing=1.15;pp.paragraph_format.space_after=Pt(2);pp.paragraph_format.space_before=Pt(2)
            if len(rows)<=7 and k<len(rows):pp.paragraph_format.keep_with_next=True
            pp.alignment=WD_ALIGN_PARAGRAPH.CENTER if len(str(txt))<26 else WD_ALIGN_PARAGRAPH.LEFT
            rr=pp.add_run(str(txt));rr.font.size=Pt(10);rr.bold=k==0
            if (red and k>0) or '____' in str(txt):rr.font.color.rgb=RGBColor.from_string('C00000')
            if k==0:
                sh=OxmlElement('w:shd');sh.set(qn('w:fill'),'F2F2F2');c._tc.get_or_add_tcPr().append(sh)
        tr=t.rows[k]._tr.get_or_add_trPr();cant=OxmlElement('w:cantSplit');tr.append(cant)
        if k==0:el=OxmlElement('w:tblHeader');tr.append(el)
    p('').paragraph_format.space_after=Pt(0)
    return t
def figure(path,caption,width=150,source=''):
    path=Path(path)
    if not path.is_absolute():path=ROOT/path
    im=Image.open(path);w,h=im.size;width=min(float(width),160,145*w/h)
    x=p(center=True);x.paragraph_format.keep_with_next=True;x.paragraph_format.space_before=Pt(5);x.paragraph_format.line_spacing=1
    shape=x.add_run().add_picture(str(path),width=Mm(width));shape._inline.docPr.set('descr',caption)
    cp=p(caption,'Caption');cp.paragraph_format.keep_with_next=bool(source);figures.append({'caption':caption,'file':str(path),'source':source})
    if source:
        x=p(source);x.paragraph_format.first_line_indent=Pt(0);x.paragraph_format.line_spacing=1.15
        for r in x.runs:r.font.size=Pt(9)

# Small native-OMML writer, supporting structured fractions, roots, matrices and scripts.
def el(n):return OxmlElement('m:'+n)
def mr(txt):
    r=el('r');pr=el('rPr');sty=el('sty');sty.set(qn('m:val'),'p');pr.append(sty);r.append(pr);t=el('t');t.text=txt;t.set(qn('xml:space'),'preserve');r.append(t);return r
def mathnodes(src):
    i=0;out=[]
    def group():
        nonlocal i
        if i>=len(src):return ''
        if src[i]!='{':v=src[i];i+=1;return v
        i+=1;start=i;d=1
        while i<len(src) and d:
            if src[i]=='{':d+=1
            if src[i]=='}':d-=1
            i+=1
        return src[start:i-1]
    while i<len(src):
        if src.startswith('\\frac',i):
            i+=5;a=group();b=group();o=el('f');nu=el('num');de=el('den');nu.extend(mathnodes(a));de.extend(mathnodes(b));o.extend([nu,de]);out.append(o)
        elif src.startswith('\\sqrt',i):
            i+=5;a=group();o=el('rad');pr=el('radPr');dh=el('degHide');dh.set(qn('m:val'),'1');pr.append(dh);o.append(pr);o.append(el('deg'));e=el('e');e.extend(mathnodes(a));o.append(e);out.append(o)
        elif src.startswith('\\mat',i):
            i+=4;a=group();d=el('d');pr=el('dPr');be=el('begChr');be.set(qn('m:val'),'[');en=el('endChr');en.set(qn('m:val'),']');pr.extend([be,en]);d.append(pr);ee=el('e');mat=el('m')
            for row in a.split(';'):
                mm=el('mr')
                for cell in row.split('|'):
                    e=el('e');e.extend(mathnodes(cell));mm.append(e)
                mat.append(mm)
            ee.append(mat);d.append(ee);out.append(d)
        elif src[i] in '_^' and out:
            token=src[i];i+=1;a=group();base=out.pop();v=el('sub' if token=='_' else 'sup');v.extend(mathnodes(a))
            if (token=='^' and base.tag==qn('m:sSub')) or (token=='_' and base.tag==qn('m:sSup')):
                o=el('sSubSup');e=base.find(qn('m:e'));old=base.find(qn('m:sub' if token=='^' else 'm:sup'));o.append(e)
                o.extend([old,v] if token=='^' else [v,old])
            else:
                o=el('sSub' if token=='_' else 'sSup');e=el('e');e.append(base);o.extend([e,v])
            out.append(o)
        else:out.append(mr(src[i]));i+=1
    return out
def eq(num,s):
    s=s.replace('\\tilde{p}','p̄')
    equations.append(num);x=p();x.paragraph_format.first_line_indent=Pt(0);x.paragraph_format.alignment=WD_ALIGN_PARAGRAPH.CENTER;x.paragraph_format.space_before=Pt(6);x.paragraph_format.space_after=Pt(6);x.paragraph_format.line_spacing=1.25
    o=el('oMath');o.extend(mathnodes(s));x._p.append(o);x.add_run('    （'+num+'）').font.size=Pt(10.5)

def csvtable(filename,title,cols,heads,widths=None):
    path=ROOT/'SC-RRT小论文编写/SC-RRT公平对比实验-v2/outputs/paper_completion_20260907/tables'/filename
    records=list(csv.DictReader(path.open(encoding='utf-8-sig')))
    names={'Dynamic-RRT (local adaptation)':'Dynamic-RRT¹','SC-RRT fixed (matched activation)':'匹配启动固定参数','SC-RRT w/o informed-sampling/feedback bundle':'关闭采样与反馈整组'}
    rows=[[names.get(r[c],r[c]) for c in cols] for r in records]
    table(title,heads,rows,widths)
def parse(filename):
    blocks=(B/filename).read_text(encoding='utf-8').split('\n\n')
    for block in blocks:
        block=block.strip()
        if not block:continue
        if block.startswith('### '):heading(block[4:],3)
        elif block.startswith('## '):heading(block[3:],2)
        elif block.startswith('# '):heading(block[2:],1)
        elif block.startswith('@gap|'):
            a=block.split('|',3);gap(a[1],a[3],a[2])
        elif block.startswith('@eq|'):
            a=block.split('|',2);eq(a[1],a[2])
        elif block.startswith('@fig|'):
            a=block.split('|');figure(a[1],a[2],float(a[3]),a[4] if len(a)>4 else '')
        elif block.startswith('@table|'):
            lines=block.splitlines();title=lines[0][7:];heads=lines[1].split('|');rows=[x.split('|') for x in lines[2:]];table(title,heads,rows)
        elif block.startswith('@csv|'):
            _,f,title,cols,heads=block.split('|');csvtable(f,title,cols.split('~'),heads.split('~'),[36]+[124/(len(heads.split('~'))-1)]*(len(heads.split('~'))-1))
        else:
            x=p(block.replace('\n',''))
            if filename=='references.txt':
                x.paragraph_format.first_line_indent=Pt(-18);x.paragraph_format.left_indent=Pt(18);x.paragraph_format.line_spacing=1.2;x.paragraph_format.space_after=Pt(6)
                x.paragraph_format.alignment=WD_ALIGN_PARAGRAPH.LEFT
                x.paragraph_format.keep_together=True
                for r in x.runs:r.font.size=Pt(10.5)
            elif filename=='appendix.txt' and ('https://' in block or '源码主要文件' in block):
                x.paragraph_format.alignment=WD_ALIGN_PARAGRAPH.LEFT

# Cover
p('上海应用技术大学',center=True).runs[0].font.size=Pt(25)
p('硕士学位论文',center=True).runs[0].font.size=Pt(24)
p('');p('')
x=p('基于视觉感知与自适应双向\n超椭球体RRT的机械臂运动规划研究','Title',True);x.paragraph_format.line_spacing=1.5
x=p('Research on Robotic Arm Motion Planning Based on Visual Perception and Adaptive Bidirectional Hyperellipsoidal RRT',center=True);x.runs[0].font.size=Pt(14)
p('');p('作者：郭兴雨',center=True);p('指导教师：吴雁  教授',center=True);p('学科专业：机械电子工程',center=True);p('学院：机械工程学院',center=True)
gap('D00','学号、研究方向、学位类别、正式提交日期及学校最新封面与声明页要求；相应位置：________。')
p('正文初稿 V1',center=True);p('2026年10月19日',center=True)
pagesection('upperRoman')
parse('abstract.txt')
heading('目录',1)
field(p(),'TOC \\o "1-3" \\h \\z','目录将在Word更新域后显示')
pagesection()
for i in range(1,7):parse(f'chapter{i}.txt')
parse('references.txt');parse('appendix.txt')
heading('B.1 OpenCV图像复用许可',2)
for chunk in (B/'assets/opencv_LICENSE.txt').read_text(encoding='utf-8').strip().split('\n\n'):
    x=p(' '.join(line.strip() for line in chunk.splitlines()));x.paragraph_format.first_line_indent=Pt(0);x.paragraph_format.line_spacing=1.1;x.paragraph_format.space_after=Pt(3)
    for r in x.runs:r.font.size=Pt(9)
heading('附录C 待补信息索引',1)
p('红色条目对应正文中的留空或待核位置。先补齐已有配置文件、原始记录和材料来源，再判断是否确有必要补做实验。待核数值不进入摘要与确定性结论。')
table('表C-1 待补与待核信息汇总',['编号','位置','需要的材料'],[[m['kind']+m['id'],m['section'],m['need']] for m in missing],[19,44,97],red=True)
heading('攻读学位期间的研究成果',1)
gap('D40','已发表、已录用或投稿论文的准确题名、作者顺序、期刊或会议、状态及证明；专利等成果如无则删除对应类别。')
heading('致谢',1)
gap('D41','请作者提供希望致谢的导师、合作人员及具体帮助，核实后用本人语气补写。')
# Add final two entries to index without rebuilding everything.
idx=doc.tables[-1]
for m in missing[-2:]:
    cells=idx.add_row().cells
    for c,txt in zip(cells,[m['kind']+m['id'],m['section'],m['need']]):
        pp=c.paragraphs[0];pp.paragraph_format.first_line_indent=Pt(0);pp.paragraph_format.line_spacing=1.15;r=pp.add_run(txt);r.font.size=Pt(10);r.font.color.rgb=RGBColor.from_string('C00000')
doc.core_properties.title='基于视觉感知与自适应双向超椭球体RRT的机械臂运动规划研究';doc.core_properties.author='郭兴雨';doc.core_properties.subject='硕士学位论文正文初稿 V1';doc.core_properties.keywords='YOLOv11-AMP;SC-RRT;视觉定位;路径规划;机械臂';doc.core_properties.comments=''
out=B.parent/'郭兴雨_硕士学位论文初稿_V1.docx';doc.save(out)
(B/'build_manifest.json').write_text(json.dumps({'output':str(out),'missing':missing,'figures':figures,'equations':equations,'tables':tables},ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({'file':str(out),'paragraphs':len(doc.paragraphs),'tables':len(doc.tables),'figures':len(figures),'equations':len(equations),'gaps':len(missing)},ensure_ascii=False))
