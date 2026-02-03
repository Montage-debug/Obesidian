#!/bin/bash
# 查看V4实验最新结果

echo "=========================================="
echo "V4实验结果查看"
echo "=========================================="
echo ""

# 查找最新的结果文件
LATEST_RESULTS=$(ls -t results/pid_results_*.csv 2>/dev/null | head -1)
LATEST_SUMMARY=$(ls -t results/analysis_summary_*.csv 2>/dev/null | head -1)
LATEST_STATS=$(ls -t results/statistical_summary_*.txt 2>/dev/null | head -1)

if [ -z "$LATEST_RESULTS" ]; then
    echo "❌ 未找到实验结果文件"
    exit 1
fi

echo "【1】实验概览"
echo "   结果文件: $(basename $LATEST_RESULTS)"
echo "   生成时间: $(stat -c %y "$LATEST_RESULTS" | cut -d'.' -f1)"
echo ""

# 统计完成情况
TOTAL=$(tail -n +2 "$LATEST_RESULTS" | wc -l)
echo "【2】完成情况: $TOTAL / 240 试验"
echo ""

# 成功率统计
echo "【3】成功率统计："
echo ""
for config in "No_PID" "Low_PID" "Optimal_PID" "High_PID"; do
    TOTAL_CFG=$(tail -n +2 "$LATEST_RESULTS" | grep "$config" | wc -l)
    SUCCESS=$(tail -n +2 "$LATEST_RESULTS" | grep "$config" | grep ",True," | wc -l)
    if [ $TOTAL_CFG -gt 0 ]; then
        SUCCESS_RATE=$(echo "scale=2; $SUCCESS * 100 / $TOTAL_CFG" | bc)
        echo "   $config: $SUCCESS/$TOTAL_CFG = $SUCCESS_RATE%"
    fi
done
echo ""

# ESR统计（新增核心指标）
echo "【4】ESR（有效采样比例）统计："
echo ""
if [ -f "$LATEST_SUMMARY" ]; then
    echo "   从摘要文件读取ESR数据..."
    cat "$LATEST_SUMMARY" | tail -n +2 | awk -F',' '{
        printf "   %s: ESR平均值 (待提取)\n", $1
    }'
else
    # 直接从原始数据计算
    echo "   No_PID ESR: $(tail -n +2 "$LATEST_RESULTS" | grep "No_PID" | grep ",True," | awk -F',' '{sum+=$18; n++} END {if(n>0) printf "%.2f%%", sum*100/n; else print "N/A"}')"
    echo "   Low_PID ESR: $(tail -n +2 "$LATEST_RESULTS" | grep "Low_PID" | grep ",True," | awk -F',' '{sum+=$18; n++} END {if(n>0) printf "%.2f%%", sum*100/n; else print "N/A"}')"
    echo "   Optimal_PID ESR: $(tail -n +2 "$LATEST_RESULTS" | grep "Optimal_PID" | grep ",True," | awk -F',' '{sum+=$18; n++} END {if(n>0) printf "%.2f%%", sum*100/n; else print "N/A"}')"
    echo "   High_PID ESR: $(tail -n +2 "$LATEST_RESULTS" | grep "High_PID" | grep ",True," | awk -F',' '{sum+=$18; n++} END {if(n>0) printf "%.2f%%", sum*100/n; else print "N/A"}')"
fi
echo ""

# 详细统计报告
if [ -f "$LATEST_STATS" ]; then
    echo "【5】详细统计报告："
    cat "$LATEST_STATS"
fi

echo ""
echo "=========================================="
echo "可视化结果："
ls -lh results/*.png | awk '{print "  " $NF}'
echo ""
echo "完整数据："
echo "  原始数据: $LATEST_RESULTS"
echo "  摘要数据: $LATEST_SUMMARY"
echo "  统计报告: $LATEST_STATS"
echo "=========================================="
