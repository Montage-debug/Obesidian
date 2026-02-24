#!/usr/bin/env python3
"""
实时监控PID参数优化实验进度
"""
import os
import sys
import time
import glob
import re
from datetime import datetime, timedelta

def parse_progress(log_content):
    """解析日志内容获取进度信息"""
    lines = log_content.split('\n')
    
    info = {
        'stage': 'Unknown',
        'current_config': 0,
        'total_configs': 0,
        'current_params': {},
        'recent_results': [],
        'avg_length': None,
        'success_rate': None
    }
    
    # 查找阶段信息
    for line in lines:
        if 'STAGE 1: COARSE GRID SEARCH' in line:
            info['stage'] = 'Stage 1 - 粗搜索'
        elif 'STAGE 2: FINE-TUNING' in line:
            info['stage'] = 'Stage 2 - 精细调优'
        elif 'STAGE 3: FINAL VALIDATION' in line:
            info['stage'] = 'Stage 3 - 最终验证'
        
        # 查找配置进度 [X/Y]
        match = re.search(r'\[(\d+)/(\d+)\].*Kp=([\d.]+).*Ki=([\d.]+).*Kd=([\d.]+)', line)
        if match:
            info['current_config'] = int(match.group(1))
            info['total_configs'] = int(match.group(2))
            info['current_params'] = {
                'Kp': float(match.group(3)),
                'Ki': float(match.group(4)),
                'Kd': float(match.group(5))
            }
        
        # 查找结果数据（路径长度）
        numbers = re.findall(r'\b\d{3,5}\b', line)
        if len(numbers) >= 3 and '->' in line:
            info['recent_results'] = [int(n) for n in numbers]
    
    # 计算平均路径长度
    if info['recent_results']:
        info['avg_length'] = sum(info['recent_results']) / len(info['recent_results'])
    
    return info

def monitor_experiment():
    """监控实验进度"""
    results_dir = "../results"
    
    print("\n" + "=" * 90)
    print("🔬 SC-RRT PID参数优化实验 - 实时监控")
    print("=" * 90)
    print(f"📁 监控目录: {results_dir}")
    print(f"⏱️  更新频率: 每5秒")
    print(f"⌨️  按 Ctrl+C 退出监控")
    print("=" * 90 + "\n")
    
    last_size = 0
    last_mtime = 0
    start_time = datetime.now()
    iteration_count = 0
    
    while True:
        try:
            iteration_count += 1
            
            # 查找最新的日志文件
            log_files = glob.glob(os.path.join(results_dir, "experiment_log_*.txt"))
            
            if not log_files:
                elapsed = datetime.now() - start_time
                print(f"\r⏳ 等待实验开始... ({int(elapsed.total_seconds())}秒)", end='', flush=True)
                time.sleep(5)
                continue
            
            latest_log = max(log_files, key=os.path.getmtime)
            current_mtime = os.path.getmtime(latest_log)
            current_size = os.path.getsize(latest_log)
            
            # 如果文件有更新或每30秒强制刷新一次
            if current_mtime != last_mtime or current_size != last_size or iteration_count % 6 == 0:
                os.system('cls' if os.name == 'nt' else 'clear')
                
                # 读取日志文件
                try:
                    with open(latest_log, 'r', encoding='utf-8') as f:
                        content = f.read()
                except:
                    time.sleep(1)
                    continue
                
                # 解析进度
                info = parse_progress(content)
                
                # 显示头部信息
                print("\n" + "=" * 90)
                print(f"🔬 SC-RRT PID参数优化实验 - 实时监控")
                print("=" * 90)
                print(f"📅 当前时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
                print(f"📁 日志文件: {os.path.basename(latest_log)} ({current_size / 1024:.1f} KB)")
                print(f"⏱️  运行时长: {str(datetime.now() - start_time).split('.')[0]}")
                print("=" * 90)
                
                # 显示当前阶段和进度
                print(f"\n📊 {info['stage']}")
                if info['total_configs'] > 0:
                    progress_pct = (info['current_config'] / info['total_configs']) * 100
                    bar_length = 50
                    filled = int(bar_length * info['current_config'] / info['total_configs'])
                    bar = '█' * filled + '░' * (bar_length - filled)
                    print(f"进度: [{bar}] {info['current_config']}/{info['total_configs']} ({progress_pct:.1f}%)")
                    
                    # 估算剩余时间
                    if info['current_config'] > 0:
                        elapsed = datetime.now() - start_time
                        avg_time_per_config = elapsed.total_seconds() / info['current_config']
                        remaining_configs = info['total_configs'] - info['current_config']
                        eta_seconds = avg_time_per_config * remaining_configs
                        eta = str(timedelta(seconds=int(eta_seconds)))
                        print(f"预计剩余: {eta}")
                
                # 显示当前参数
                if info['current_params']:
                    print(f"\n🎯 当前配置:")
                    print(f"   Kp = {info['current_params']['Kp']:.3f}")
                    print(f"   Ki = {info['current_params']['Ki']:.3f}")
                    print(f"   Kd = {info['current_params']['Kd']:.3f}")
                
                # 显示最近结果
                if info['recent_results']:
                    print(f"\n📈 最近试验结果 (路径长度):")
                    print(f"   {', '.join(map(str, info['recent_results']))}")
                    if info['avg_length']:
                        print(f"   平均: {info['avg_length']:.1f}")
                
                # 显示最后几行日志
                print("\n" + "─" * 90)
                print("📝 最新日志 (最后15行):")
                print("─" * 90)
                lines = content.split('\n')
                recent_lines = [l for l in lines[-15:] if l.strip()]
                for line in recent_lines:
                    # 截断过长的行
                    if len(line) > 88:
                        line = line[:85] + "..."
                    print(line)
                
                print("\n" + "=" * 90)
                print("⌨️  按 Ctrl+C 退出监控 | 自动刷新: 5秒")
                print("=" * 90)
                
                last_mtime = current_mtime
                last_size = current_size
            
            time.sleep(5)
            
        except KeyboardInterrupt:
            print("\n\n✅ 监控已停止")
            break
        except Exception as e:
            print(f"\n❌ 错误: {e}")
            time.sleep(5)

if __name__ == "__main__":
    monitor_experiment()
