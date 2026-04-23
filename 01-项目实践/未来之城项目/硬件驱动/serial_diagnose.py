#!/usr/bin/env python3
"""
串口数据流诊断工具
- 精确记录每次 read() 的时间戳和字节内容
- 无数据时记录超时事件（100ms timeout）
- 用于诊断设备拔出后数据何时真正停止

用法：先停止 ROS 节点，然后运行此脚本，接上/拔掉手柄观察输出。
"""
import serial
import time
import sys

DEVICE = "/dev/tty_massage_head_manager"
BAUD = 115200
READ_TIMEOUT = 0.1  # 100ms 超时

def main():
    print(f"[诊断] 打开串口 {DEVICE} @ {BAUD} baud, 读超时={READ_TIMEOUT}s")
    try:
        ser = serial.Serial(DEVICE, BAUD, timeout=READ_TIMEOUT)
    except Exception as e:
        print(f"[错误] 无法打开串口: {e}")
        sys.exit(1)
    
    print("[诊断] 串口已打开，开始监控数据流...")
    print("[诊断] 请插入/拔出手柄设备，观察数据流变化")
    print("-" * 80)
    
    last_data_time = None
    consecutive_timeouts = 0
    total_packets = 0
    
    try:
        while True:
            now = time.time()
            data = ser.read(256)
            
            if data:
                hex_str = data.hex(' ')
                elapsed_since_last = ""
                if last_data_time:
                    gap_ms = (now - last_data_time) * 1000
                    elapsed_since_last = f" (距上次: {gap_ms:.1f}ms)"
                
                last_data_time = now
                total_packets += 1
                consecutive_timeouts = 0
                
                ts = time.strftime('%H:%M:%S', time.localtime(now))
                ms = f"{now % 1:.3f}"[1:]
                print(f"[{ts}{ms}] 📥 {len(data):3d}字节: {hex_str}{elapsed_since_last}")
            else:
                consecutive_timeouts += 1
                if last_data_time:
                    gap_ms = (now - last_data_time) * 1000
                    ts = time.strftime('%H:%M:%S', time.localtime(now))
                    ms = f"{now % 1:.3f}"[1:]
                    
                    if consecutive_timeouts <= 3:
                        print(f"[{ts}{ms}] ⏱️  无数据 (已{gap_ms:.0f}ms无数据, 连续超时#{consecutive_timeouts})")
                    elif consecutive_timeouts == 4:
                        print(f"[{ts}{ms}] 🔴 设备疑似离线 ({gap_ms:.0f}ms无数据)")
                    elif consecutive_timeouts % 10 == 0:
                        print(f"[{ts}{ms}] 🔴 持续无数据 ({gap_ms:.0f}ms, 超时#{consecutive_timeouts})")
                else:
                    if consecutive_timeouts == 1:
                        ts = time.strftime('%H:%M:%S', time.localtime(now))
                        ms = f"{now % 1:.3f}"[1:]
                        print(f"[{ts}{ms}] ⏱️  等待首次数据...")
                    elif consecutive_timeouts % 20 == 0:
                        print(f"  ... 继续等待 (超时#{consecutive_timeouts})")
                        
    except KeyboardInterrupt:
        print(f"\n[诊断] 结束。共收到 {total_packets} 次有效数据。")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
