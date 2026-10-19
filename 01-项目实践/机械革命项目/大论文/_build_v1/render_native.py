"""Use the packaged rasterizer with native Word PDF conversion on Windows.
The selected Codex Windows runtime declares libreOfficeVersion=null.
"""
import os,sys,importlib.util,shutil
from pathlib import Path
PKG=Path(r'C:\Users\26434\.codex\plugins\cache\openai-primary-runtime\documents\26.1015.12040\skills\documents')
POP=Path(r'C:\Users\26434\.cache\codex-runtimes\codex-primary-runtime\dependencies\native\poppler\Library\bin')
os.environ['PATH']=str(POP)+os.pathsep+os.environ['PATH']
spec=importlib.util.spec_from_file_location('packaged_render_docx',PKG/'render_docx.py')
renderer=importlib.util.module_from_spec(spec);spec.loader.exec_module(renderer)
pdf=Path(sys.argv[2]).resolve()
def native_pdf(input_path,user_profile,convert_tmp_dir,stem,verbose=False):
 target=Path(convert_tmp_dir)/(stem+'.pdf');shutil.copy2(pdf,target)
 return str(target),'Native Microsoft Word PDF export used; no bundled LibreOffice exists in this Windows runtime.'
renderer.convert_to_pdf=native_pdf
pages=renderer.rasterize(sys.argv[1],sys.argv[3],int(sys.argv[4]) if len(sys.argv)>4 else 110,False,False)
print('Rendered',len(pages),'pages')
