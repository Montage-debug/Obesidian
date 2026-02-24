# -*- coding: utf-8 -*-
"""
消融实验实时监控脚本
"""
import time
from pathlib import Path
import re

def monitor_ablation():
    """监控消融实验进度"""
    results_dir = Path('results/ablation')
    
    # 找到最新的日志文件
    log_files = list(results_dir.glob('ablation_log_*.txt'))
    if not log_files:
        print("未找到日志文件")
        return
    
    latest_log = max(log_files, key=lambda p: p.stat().st_mtime)
    print(f"监控日志: {latest_log.name}\n")
    
    last_position = 0
    
    try:
        while True:
            with open(latest_log, 'r', encoding='utf-8') as f:
                f.seek(last_position)
                new_content = f.read()
                if new_content:
                    print(new_content, end='', flush=True)
                    last_position = f.tell()
            
            time.sleep(2)
    except KeyboardInterrupt:
        print("\n\n监控已停止")

if __name__ == '__main__':
    monitor_ablation()
