from pathlib import Path
from pypdf import PdfReader
from PIL import Image,ImageDraw
import json,re,sys
B=Path(__file__).parent
pdf=PdfReader(B/(sys.argv[1] if len(sys.argv)>1 else 'draft.pdf'))
out=[]
for i,p in enumerate(pdf.pages):
    t=p.extract_text() or '';out.append({'page':i+1,'chars':len(t),'start':t[:160],'end':t[-130:]})
(B/'page_text.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(out,ensure_ascii=False))
