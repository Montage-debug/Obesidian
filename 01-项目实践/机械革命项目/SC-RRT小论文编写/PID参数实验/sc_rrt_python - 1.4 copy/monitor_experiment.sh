#!/bin/bash
# 实验监控脚本

echo "=========================================="
echo "PID参数实验监控"
echo "=========================================="
echo ""

# 1. 检查进程状态
echo "【1】进程状态："
ps aux | grep "[p]ython.*run_pid_experiment" | head -1
if [ $? -eq 0 ]; then
    echo "✅ 实验正在运行"
    PID=$(ps aux | grep "[p]ython.*run_pid_experiment" | awk '{print $2}')
    echo "   进程ID: $PID"
    echo "   CPU使用: $(ps aux | grep "[p]ython.*run_pid_experiment" | awk '{print $3}')%"
    echo "   内存使用: $(ps aux | grep "[p]ython.*run_pid_experiment" | awk '{print $4}')%"
    echo "   运行时间: $(ps aux | grep "[p]ython.*run_pid_experiment" | awk '{print $10}')"
else
    echo "❌ 实验未运行"
fi
echo ""

# 2. 检查日志文件
echo "【2】日志文件："
if [ -f "v4_experiment.log" ]; then
    LOG_SIZE=$(du -h v4_experiment.log | awk '{print $1}')
    LOG_LINES=$(wc -l < v4_experiment.log)
    echo "   文件大小: $LOG_SIZE"
    echo "   日志行数: $LOG_LINES"
    echo ""
    echo "   最新10行："
    tail -10 v4_experiment.log | sed 's/^/     /'
else
    echo "   ❌ 日志文件不存在"
fi
echo ""

# 3. 检查结果文件
echo "【3】结果文件："
if [ -d "results" ]; then
    LATEST_CSV=$(ls -t results/pid_results_*.csv 2>/dev/null | head -1)
    if [ -n "$LATEST_CSV" ]; then
        echo "   最新结果: $(basename $LATEST_CSV)"
        echo "   生成时间: $(stat -c %y "$LATEST_CSV" | cut -d'.' -f1)"
        echo "   文件大小: $(du -h "$LATEST_CSV" | awk '{print $1}')"
        
        # 统计完成的试验数
        COMPLETED=$(tail -n +2 "$LATEST_CSV" | wc -l)
        echo "   已完成试验: $COMPLETED / 240"
        echo "   完成进度: $(echo "scale=1; $COMPLETED * 100 / 240" | bc)%"
    else
        echo "   ⏳ 等待结果生成..."
    fi
else
    echo "   ❌ 结果目录不存在"
fi
echo ""

# 4. 快速统计（如果有结果）
if [ -n "$LATEST_CSV" ] && [ -f "$LATEST_CSV" ]; then
    echo "【4】当前统计："
    echo "   No_PID成功率: $(tail -n +2 "$LATEST_CSV" | grep "No_PID" | grep -c ",True,") / $(tail -n +2 "$LATEST_CSV" | grep -c "No_PID")"
    echo "   Low_PID成功率: $(tail -n +2 "$LATEST_CSV" | grep "Low_PID" | grep -c ",True,") / $(tail -n +2 "$LATEST_CSV" | grep -c "Low_PID")"
    echo "   Optimal_PID成功率: $(tail -n +2 "$LATEST_CSV" | grep "Optimal_PID" | grep -c ",True,") / $(tail -n +2 "$LATEST_CSV" | grep -c "Optimal_PID")"
    echo "   High_PID成功率: $(tail -n +2 "$LATEST_CSV" | grep "High_PID" | grep -c ",True,") / $(tail -n +2 "$LATEST_CSV" | grep -c "High_PID")"
fi

echo ""
echo "=========================================="
echo "提示："
echo "  实时监控: tail -f v4_experiment.log"
echo "  查看进程: ps aux | grep run_pid_experiment"
echo "  重新运行: bash monitor_experiment.sh"
echo "=========================================="
