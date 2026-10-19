from pathlib import Path
import re

out = Path(__file__).resolve().parent.parent
root = out.parent.parent
source = root / '版本/2026-09-09-通用技术简历-v03/郭兴雨-通用技术简历-单页.html'
text = source.read_text(encoding='utf-8')
body = (out / '核对材料/机械正文.html').read_text(encoding='utf-8')
text, count = re.subn(r'<p class="summary">.*?</main>', lambda _: body + '</main>', text, count=1, flags=re.S)
assert count == 1
text = re.sub(r'<title>.*?</title>', '<title>郭兴雨｜机械设计与机电研发简历</title>', text)
text = text.replace('机械电子工程硕士 · 2027 届', '机械设计 / 机电研发 · 2027 届')
text = re.sub(r"const storageKey='[^']+';", "const storageKey='resume-mechanical-gxy-20260909-v04';", text, count=1)
text = text.replace("const newKey='resume-soe-gxy-'", "const newKey='resume-mechanical-gxy-'")
text = text.replace('郭兴雨-通用技术简历-单页.html', '郭兴雨-机械设计与机电研发-单页简历.html')
# Use the address explicitly confirmed for this mechanical application edition.
text = text.replace('gxyhaoxianghenli@gmail.com', 'gxyhaoxianghenlihai@gmail.com')
migration = "// Apply the user's role correction to earlier browser saves.\nfor(const company of resume.querySelectorAll('.company strong')){\n  if(company.textContent.trim()==='杭州市云深处科技有限公司｜测试实习生')company.textContent='杭州市云深处科技有限公司｜研发实习生';\n}\n"
text = text.replace('function persist()', migration + 'function persist()', 1)
text = text.replace('</style>', '''
/* Mechanical edition: larger body text with readable spacing. */
body{font-size:11pt;line-height:1.4}
h2{margin:9px 0 5px}
h3{font-size:10.4pt}
.company strong{font-size:10.3pt}
.edu strong{font-size:10pt}
li{margin:2.5px 0}
.mechanical-skills p{font-size:9.8pt;line-height:1.4;margin:3px 0}
.mechanical-skills b{color:var(--blue)}
.research p:not(.paper-title){font-size:10.4pt}
.paper-title{font-size:9.4pt;line-height:1.32;text-align:left;overflow-wrap:normal}
</style>''')
(out / '郭兴雨-机械设计与机电研发-单页简历.html').write_text(text, encoding='utf-8')
print('Mechanical HTML built.')
