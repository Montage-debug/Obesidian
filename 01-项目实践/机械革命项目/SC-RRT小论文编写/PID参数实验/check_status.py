#!/usr/bin/env python3
"""快速检查实验状态"""
import os
import glob
from datetime import datetime

results_dir = "results"
log_files = glob.glob(os.path.join(results_dir, "experiment_log_*.txt"))

if not log_files:
    print("❌ 实验尚未开始（没有日志文件）")
else:
    latest_log = max(log_files, key=os.path.getmtime)
    size = os.path.getsize(latest_log)
    mtime = datetime.fromtimestamp(os.path.getmtime(latest_log))
    
    print(f"✅ 实验正在运行")
    print(f"📁 日志: {os.path.basename(latest_log)}")
    print(f"📊 大小: {size/1024:.1f} KB")
    print(f"🕒 最后更新: {mtime.strftime('%H:%M:%S')}")
    
    # 读取最后几行
    with open(latest_log, 'r', encoding='utf-8') as f:
        lines = f.readlines()
        print(f"\n📝 最后5行:")
        for line in lines[-5:]:
            print(f"  {line.rstrip()}")
