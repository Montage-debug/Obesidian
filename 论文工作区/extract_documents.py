# -*- coding: utf-8 -*-
"""
提取PDF和Word文档内容
"""
import fitz  # PyMuPDF
from docx import Document
import os

def extract_pdf_text(pdf_path):
    """提取PDF文本内容"""
    try:
        doc = fitz.open(pdf_path)
        text = ""
        for page_num in range(len(doc)):
            page = doc[page_num]
            text += f"\n\n=== 第{page_num+1}页 ===\n"
            text += page.get_text()
        doc.close()
        return text
    except Exception as e:
        return f"错误: {str(e)}"

def extract_word_text(docx_path):
    """提取Word文档内容"""
    try:
        doc = Document(docx_path)
        text = ""
        for para in doc.paragraphs:
            if para.text.strip():
                text += para.text + "\n"
        return text
    except Exception as e:
        return f"错误: {str(e)}"

if __name__ == "__main__":
    base_dir = r"e:\obsidian_知识库\论文工作区"
    
    # PDF文件列表
    pdf_files = [
        "A multi strategy bidirectional RRT .pdf",
        "Adaptive Goal-Biased Bi-RRT for Online Path Planning ofRobotic Manipulators.pdf",
        "Improved Algorithm of RRT Path Planning and theApplication in Complex Environme(1).pdf"
    ]
    
    # Word文档
    word_file = "第五版SC-RRT，一种基于自适应双向超椭球体采样约束的路径规划研究.docx"
    
    # 提取PDF内容
    print("=" * 80)
    print("提取PDF文档内容")
    print("=" * 80)
    for i, pdf_file in enumerate(pdf_files, 1):
        pdf_path = os.path.join(base_dir, pdf_file)
        print(f"\n\n{'=' * 80}")
        print(f"PDF文档 {i}: {pdf_file}")
        print("=" * 80)
        if os.path.exists(pdf_path):
            text = extract_pdf_text(pdf_path)
            # 只输出前5000个字符作为摘要
            print(text[:5000] if len(text) > 5000 else text)
            print(f"\n[总字符数: {len(text)}]")
        else:
            print(f"文件不存在: {pdf_path}")
    
    # 提取Word内容
    print("\n\n" + "=" * 80)
    print("提取Word文档内容")
    print("=" * 80)
    word_path = os.path.join(base_dir, word_file)
    if os.path.exists(word_path):
        text = extract_word_text(word_path)
        print(text)
        print(f"\n[总字符数: {len(text)}]")
    else:
        print(f"文件不存在: {word_path}")
