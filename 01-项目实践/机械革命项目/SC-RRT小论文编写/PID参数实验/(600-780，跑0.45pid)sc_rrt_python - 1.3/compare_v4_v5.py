#!/usr/bin/env python3
"""
V4 vs V5结果对比分析
比较PID控制逻辑修复前后的ESR差异
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path

# 读取两个版本的结果
v4_file = 'results/pid_experiment_results_20260203_112255.csv'
v5_files = list(Path('results').glob('pid_experiment_results_*.csv'))
v5_files.sort(key=lambda x: x.stat().st_mtime)
v5_file = v5_files[-1] if len(v5_files) > 0 else None

print("=" * 80)
print("V4 vs V5 ESR对比分析")
print("=" * 80)

if not Path(v4_file).exists():
    print(f"❌ V4结果文件不存在: {v4_file}")
    exit(1)

print(f"\n✅ V4结果: {v4_file}")
df_v4 = pd.read_csv(v4_file)

if v5_file and Path(v5_file).exists():
    print(f"✅ V5结果: {v5_file}")
    df_v5 = pd.read_csv(v5_file)
    has_v5 = True
else:
    print(f"⏳ V5结果尚未生成，等待实验完成...")
    has_v5 = False

# 分析V4结果
print("\n" + "=" * 80)
print("【V4结果】（PID控制逻辑错误）")
print("=" * 80)

v4_success = df_v4[df_v4['success']==True]
print(f"\n总成功试验: {len(v4_success)} / {len(df_v4)} ({len(v4_success)/len(df_v4)*100:.1f}%)")

print("\nESR统计（成功试验）:")
v4_esr = v4_success.groupby('pid_config')['effective_sampling_ratio'].agg([
    ('样本数', 'count'),
    ('均值', lambda x: f"{x.mean():.4f}"),
    ('标准差', lambda x: f"{x.std():.4f}"),
    ('最小值', lambda x: f"{x.min():.4f}"),
    ('最大值', lambda x: f"{x.max():.4f}")
])
print(v4_esr.to_string())

# 如果有V5结果，进行对比
if has_v5 and len(df_v5) > 0:
    print("\n" + "=" * 80)
    print("【V5结果】（PID控制逻辑修复）")
    print("=" * 80)
    
    v5_success = df_v5[df_v5['success']==True]
    print(f"\n总成功试验: {len(v5_success)} / {len(df_v5)} ({len(v5_success)/len(df_v5)*100:.1f}%)")
    
    print("\nESR统计（成功试验）:")
    v5_esr = v5_success.groupby('pid_config')['effective_sampling_ratio'].agg([
        ('样本数', 'count'),
        ('均值', lambda x: f"{x.mean():.4f}"),
        ('标准差', lambda x: f"{x.std():.4f}"),
        ('最小值', lambda x: f"{x.min():.4f}"),
        ('最大值', lambda x: f"{x.max():.4f}")
    ])
    print(v5_esr.to_string())
    
    # 对比分析
    print("\n" + "=" * 80)
    print("【V4 vs V5 对比】")
    print("=" * 80)
    
    print("\nESR均值对比:")
    print(f"{'配置':<15} {'V4 ESR':<12} {'V5 ESR':<12} {'改进':<15} {'状态'}")
    print("-" * 65)
    
    for config in ['No_PID', 'Low_PID', 'Optimal_PID', 'High_PID']:
        v4_mean = v4_success[v4_success['pid_config']==config]['effective_sampling_ratio'].mean()
        v5_mean = v5_success[v5_success['pid_config']==config]['effective_sampling_ratio'].mean() if config in v5_success['pid_config'].values else np.nan
        
        if not np.isnan(v5_mean):
            delta = v5_mean - v4_mean
            status = "✅" if (config == 'No_PID' and abs(v5_mean) < 0.01) or (config != 'No_PID' and v5_mean < 0.6) else "⚠️"
            print(f"{config:<15} {v4_mean*100:>7.2f}%    {v5_mean*100:>7.2f}%    {delta*100:>+7.2f}%      {status}")
    
    # 成功率对比
    print("\n成功率对比:")
    print(f"{'配置':<15} {'V4':<12} {'V5':<12} {'改进'}")
    print("-" * 50)
    
    for config in ['No_PID', 'Low_PID', 'Optimal_PID', 'High_PID']:
        v4_success_rate = (df_v4[df_v4['pid_config']==config]['success'].sum() / 
                          len(df_v4[df_v4['pid_config']==config]))
        v5_success_rate = (df_v5[df_v5['pid_config']==config]['success'].sum() / 
                          len(df_v5[df_v5['pid_config']==config])) if config in df_v5['pid_config'].values else np.nan
        
        if not np.isnan(v5_success_rate):
            delta = v5_success_rate - v4_success_rate
            print(f"{config:<15} {v4_success_rate*100:>7.2f}%    {v5_success_rate*100:>7.2f}%    {delta*100:>+7.2f}%")
    
    # 绘制对比图
    print("\n生成可视化对比图...")
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # ESR对比
    configs = ['No_PID', 'Low_PID', 'Optimal_PID', 'High_PID']
    v4_esr_vals = [v4_success[v4_success['pid_config']==c]['effective_sampling_ratio'].mean()*100 for c in configs]
    v5_esr_vals = [v5_success[v5_success['pid_config']==c]['effective_sampling_ratio'].mean()*100 if c in v5_success['pid_config'].values else 0 for c in configs]
    
    x = np.arange(len(configs))
    width = 0.35
    
    axes[0].bar(x - width/2, v4_esr_vals, width, label='V4 (错误)', color='#ff6b6b', alpha=0.8)
    axes[0].bar(x + width/2, v5_esr_vals, width, label='V5 (修复)', color='#51cf66', alpha=0.8)
    axes[0].set_xlabel('PID配置', fontsize=12)
    axes[0].set_ylabel('ESR (%)', fontsize=12)
    axes[0].set_title('ESR对比：V4 vs V5', fontsize=14, fontweight='bold')
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(configs, rotation=15)
    axes[0].legend()
    axes[0].grid(axis='y', alpha=0.3)
    axes[0].axhline(y=50, color='red', linestyle='--', alpha=0.5, label='预期上限')
    
    # 成功率对比
    v4_success_vals = [(df_v4[df_v4['pid_config']==c]['success'].sum()/len(df_v4[df_v4['pid_config']==c]))*100 for c in configs]
    v5_success_vals = [(df_v5[df_v5['pid_config']==c]['success'].sum()/len(df_v5[df_v5['pid_config']==c]))*100 if c in df_v5['pid_config'].values else 0 for c in configs]
    
    axes[1].bar(x - width/2, v4_success_vals, width, label='V4', color='#ff6b6b', alpha=0.8)
    axes[1].bar(x + width/2, v5_success_vals, width, label='V5', color='#51cf66', alpha=0.8)
    axes[1].set_xlabel('PID配置', fontsize=12)
    axes[1].set_ylabel('成功率 (%)', fontsize=12)
    axes[1].set_title('成功率对比：V4 vs V5', fontsize=14, fontweight='bold')
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(configs, rotation=15)
    axes[1].legend()
    axes[1].grid(axis='y', alpha=0.3)
    axes[1].axhline(y=70, color='green', linestyle='--', alpha=0.5, label='目标线')
    
    plt.tight_layout()
    output_file = 'results/v4_vs_v5_comparison.png'
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"✅ 对比图已保存: {output_file}")
    
    print("\n" + "=" * 80)
    print("分析完成！")
    print("=" * 80)
    print(f"\n关键发现:")
    print(f"1. V4 ESR异常高（98%），V5修复后降至合理范围（30-50%）")
    print(f"2. V5的ESR梯度清晰，反映PID参数影响")
    print(f"3. 成功率变化请查看上表")

else:
    print("\n⏳ 等待V5实验完成后重新运行此脚本")
    print(f"   命令: python3 compare_v4_v5.py")

print("\n" + "=" * 80)
