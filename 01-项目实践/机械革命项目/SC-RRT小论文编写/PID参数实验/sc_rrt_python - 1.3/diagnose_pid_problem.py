"""
PID实验问题诊断与改进建议
分析为什么No_PID表现最好，并提供改进方案
"""

import pandas as pd
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib import font_manager
import seaborn as sns

# 中文字体配置
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

def load_latest_results():
    """加载最新的实验结果"""
    results_dir = Path("results")
    csv_files = list(results_dir.glob("pid_results_*.csv"))
    if not csv_files:
        raise FileNotFoundError("未找到实验结果文件")
    latest_file = max(csv_files, key=lambda x: x.stat().st_mtime)
    print(f"加载结果文件: {latest_file}")
    return pd.read_csv(latest_file)

def analyze_problem(df):
    """深度分析为什么No_PID表现最好"""
    
    print("\n" + "="*70)
    print("问题诊断：为什么 No_PID 成功率最高？")
    print("="*70)
    
    # 1. 成功率对比
    print("\n【1】成功率分析")
    print("-" * 50)
    success_rate = df.groupby('pid_config')['success'].agg(['mean', 'sum', 'count'])
    success_rate.columns = ['成功率', '成功次数', '总次数']
    success_rate['成功率%'] = success_rate['成功率'] * 100
    success_rate = success_rate.sort_values('成功率', ascending=False)
    print(success_rate[['成功率%', '成功次数', '总次数']])
    
    # 2. 采样效率分析
    print("\n【2】有效采样比例(ESR)分析 - PID是否过度约束？")
    print("-" * 50)
    esr_stats = df.groupby('pid_config')['effective_sampling_ratio'].agg(['mean', 'std'])
    esr_stats.columns = ['ESR均值', 'ESR标准差']
    esr_stats['ESR均值%'] = esr_stats['ESR均值'] * 100
    esr_stats = esr_stats.sort_values('ESR均值', ascending=False)
    print(esr_stats)
    
    # 关键发现：ESR vs 成功率的关系
    pid_configs = df['pid_config'].unique()
    esr_vs_success = []
    for config in pid_configs:
        config_data = df[df['pid_config'] == config]
        esr_mean = config_data['effective_sampling_ratio'].mean()
        success_rate = config_data['success'].mean()
        esr_vs_success.append({
            'config': config,
            'ESR': esr_mean * 100,
            '成功率': success_rate * 100
        })
    esr_df = pd.DataFrame(esr_vs_success).sort_values('成功率', ascending=False)
    print("\nESR vs 成功率关系：")
    print(esr_df)
    
    # 3. 迭代次数分析
    print("\n【3】迭代次数分析 - PID是否影响收敛速度？")
    print("-" * 50)
    df_success = df[df['success'] == True].copy()
    if len(df_success) > 0:
        iter_stats = df_success.groupby('pid_config')['iterations'].agg(['mean', 'median', 'std'])
        iter_stats.columns = ['均值', '中位数', '标准差']
        iter_stats = iter_stats.sort_values('均值')
        print(iter_stats)
    
    # 4. 路径质量分析（仅成功案例）
    print("\n【4】路径质量分析（成功案例）")
    print("-" * 50)
    if len(df_success) > 0:
        quality_metrics = df_success.groupby('pid_config').agg({
            'path_length': ['mean', 'std'],
            'planning_time': ['mean', 'std'],
            'tree_nodes': ['mean', 'std']
        })
        print(quality_metrics)
    
    # 5. 失败原因分析
    print("\n【5】失败原因分析")
    print("-" * 50)
    df_failed = df[df['success'] == False].copy()
    failure_rate = df_failed.groupby('pid_config').size() / df.groupby('pid_config').size()
    failure_rate = failure_rate.sort_values(ascending=False)
    print("失败率排序：")
    print(failure_rate * 100)
    
    # 6. 关键问题识别
    print("\n" + "="*70)
    print("【核心问题识别】")
    print("="*70)
    
    no_pid_success = df[df['pid_config'] == 'No_PID']['success'].mean() * 100
    pid_avg_success = df[df['pid_config'] != 'No_PID']['success'].mean() * 100
    
    print(f"\n1. No_PID成功率: {no_pid_success:.1f}%")
    print(f"   PID平均成功率: {pid_avg_success:.1f}%")
    print(f"   差距: {no_pid_success - pid_avg_success:.1f}%")
    
    # ESR分析
    no_pid_esr = df[df['pid_config'] == 'No_PID']['effective_sampling_ratio'].mean() * 100
    pid_avg_esr = df[(df['pid_config'] != 'No_PID') & (df['pid_config'] != 'Fixed_Ellipsoid')]['effective_sampling_ratio'].mean() * 100
    print(f"\n2. No_PID的ESR: {no_pid_esr:.2f}% (无椭球约束)")
    print(f"   PID平均ESR: {pid_avg_esr:.2f}% (有椭球约束)")
    
    if no_pid_esr == 0 and pid_avg_esr > 0:
        print("\n   ⚠️ 关键发现：No_PID完全自由采样，PID被椭球约束限制！")
        print("   可能问题：")
        print("   - PID椭球约束过于严格，排除了潜在的好路径")
        print("   - 椭球参数设置不当，覆盖范围不足")
        print("   - PID调节导致椭球收缩过快，失去探索能力")
    
    # 迭代效率分析
    if len(df_success) > 0:
        no_pid_iter = df_success[df_success['pid_config'] == 'No_PID']['iterations'].mean()
        pid_iter = df_success[(df_success['pid_config'] != 'No_PID') & 
                             (df_success['pid_config'] != 'Fixed_Ellipsoid')]['iterations'].mean()
        print(f"\n3. 平均迭代次数（成功案例）：")
        print(f"   No_PID: {no_pid_iter:.0f}")
        print(f"   PID平均: {pid_iter:.0f}")
        if pid_iter > no_pid_iter:
            print(f"   ⚠️ PID需要更多迭代，差异: +{pid_iter-no_pid_iter:.0f}次")
    
    return esr_df

def generate_diagnostic_plots(df):
    """生成诊断图表"""
    print("\n" + "="*70)
    print("生成诊断可视化")
    print("="*70)
    
    output_dir = Path("results/diagnosis")
    output_dir.mkdir(exist_ok=True)
    
    # 图1：成功率 vs ESR 散点图
    print("\n生成成功率-ESR关系图...")
    fig, ax = plt.subplots(figsize=(10, 6))
    
    for config in df['pid_config'].unique():
        config_data = df[df['pid_config'] == config]
        success_rate = config_data['success'].mean() * 100
        esr_mean = config_data['effective_sampling_ratio'].mean() * 100
        
        color = 'red' if config == 'No_PID' else 'blue' if 'Fixed' in config else 'green'
        marker = 's' if config == 'No_PID' else 'o'
        size = 200 if config == 'No_PID' else 100
        
        ax.scatter(esr_mean, success_rate, s=size, alpha=0.7, 
                  color=color, marker=marker, label=config)
    
    ax.set_xlabel('有效采样比例 ESR (%)', fontsize=12)
    ax.set_ylabel('成功率 (%)', fontsize=12)
    ax.set_title('成功率 vs 有效采样比例 (ESR)\n关键问题：PID约束是否过度？', 
                fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    plt.tight_layout()
    plt.savefig(output_dir / 'success_vs_esr.png', dpi=300, bbox_inches='tight')
    plt.close()
    print(f"已保存: {output_dir / 'success_vs_esr.png'}")
    
    # 图2：不同配置的迭代效率对比
    print("\n生成采样效率对比图...")
    df_success = df[df['success'] == True].copy()
    
    if len(df_success) > 0:
        fig, axes = plt.subplots(1, 2, figsize=(14, 5))
        
        # 左图：迭代次数
        ax1 = axes[0]
        iter_data = df_success.groupby('pid_config')['iterations'].mean().sort_values()
        colors = ['red' if 'No_PID' in idx else 'skyblue' for idx in iter_data.index]
        iter_data.plot(kind='barh', ax=ax1, color=colors)
        ax1.set_xlabel('平均迭代次数', fontsize=11)
        ax1.set_title('达到成功所需的平均迭代次数\n(越低越好)', fontsize=12, fontweight='bold')
        ax1.grid(True, axis='x', alpha=0.3)
        
        # 右图：规划时间
        ax2 = axes[1]
        time_data = df_success.groupby('pid_config')['planning_time'].mean().sort_values()
        colors = ['red' if 'No_PID' in idx else 'lightcoral' for idx in time_data.index]
        time_data.plot(kind='barh', ax=ax2, color=colors)
        ax2.set_xlabel('平均规划时间 (秒)', fontsize=11)
        ax2.set_title('规划时间对比\n(越低越好)', fontsize=12, fontweight='bold')
        ax2.grid(True, axis='x', alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(output_dir / 'efficiency_comparison.png', dpi=300, bbox_inches='tight')
        plt.close()
        print(f"已保存: {output_dir / 'efficiency_comparison.png'}")

def generate_improvement_suggestions(df):
    """生成改进建议"""
    print("\n" + "="*70)
    print("改进建议")
    print("="*70)
    
    suggestions = []
    
    # 分析1：ESR过低
    pid_data = df[(df['pid_config'] != 'No_PID') & (df['pid_config'] != 'Fixed_Ellipsoid')]
    avg_esr = pid_data['effective_sampling_ratio'].mean()
    
    if avg_esr < 0.5:  # ESR低于50%
        suggestions.append({
            'priority': '高',
            'problem': f'PID椭球约束过严（平均ESR={avg_esr*100:.1f}%）',
            'suggestion': '放宽椭球约束参数',
            'actions': [
                '1. 增大 gamma_max (当前建议: 从6.0提高到8.0-10.0)',
                '2. 增大 p_max (当前建议: 从0.6提高到0.8)',
                '3. 降低 ellipsoid_buffer 系数',
                '4. 调整PID参数使椭球收缩更平缓'
            ]
        })
    
    # 分析2：成功率低
    no_pid_success = df[df['pid_config'] == 'No_PID']['success'].mean()
    pid_avg_success = pid_data['success'].mean()
    
    if no_pid_success > pid_avg_success * 1.1:  # No_PID成功率高出10%以上
        suggestions.append({
            'priority': '高',
            'problem': f'PID配置成功率显著低于No_PID ({pid_avg_success*100:.1f}% vs {no_pid_success*100:.1f}%)',
            'suggestion': '重新设计PID参数范围',
            'actions': [
                '1. 测试更小的PID增益 (Kp < 0.1)',
                '2. 测试更大的Ki增益，加快收敛',
                '3. 考虑使用自适应PID增益',
                '4. 增加"渐进式PID"模式：初期弱约束，后期强约束'
            ]
        })
    
    # 分析3：迭代效率
    df_success = df[df['success'] == True]
    if len(df_success) > 0:
        no_pid_iter = df_success[df_success['pid_config'] == 'No_PID']['iterations'].mean()
        pid_iter = df_success[(df_success['pid_config'] != 'No_PID') & 
                            (df_success['pid_config'] != 'Fixed_Ellipsoid')]['iterations'].mean()
        
        if pid_iter > no_pid_iter * 1.2:  # PID需要更多迭代
            suggestions.append({
                'priority': '中',
                'problem': f'PID模式需要更多迭代才能成功 ({pid_iter:.0f} vs {no_pid_iter:.0f})',
                'suggestion': '优化采样策略',
                'actions': [
                    '1. 在椭球内使用更智能的采样策略（如梯度引导）',
                    '2. 结合局部优化方法',
                    '3. 动态调整椭球位置和形状',
                    '4. 增加"混合采样"：部分自由采样+部分椭球采样'
                ]
            })
    
    # 输出建议
    for i, sug in enumerate(suggestions, 1):
        print(f"\n【建议 {i}】优先级: {sug['priority']}")
        print(f"问题: {sug['problem']}")
        print(f"建议: {sug['suggestion']}")
        print("具体行动:")
        for action in sug['actions']:
            print(f"  {action}")
    
    # 保存建议到文件
    output_dir = Path("results/diagnosis")
    output_dir.mkdir(exist_ok=True)
    
    with open(output_dir / 'improvement_suggestions.txt', 'w', encoding='utf-8') as f:
        f.write("="*70 + "\n")
        f.write("PID实验改进建议\n")
        f.write("="*70 + "\n\n")
        
        for i, sug in enumerate(suggestions, 1):
            f.write(f"【建议 {i}】优先级: {sug['priority']}\n")
            f.write(f"问题: {sug['problem']}\n")
            f.write(f"建议: {sug['suggestion']}\n")
            f.write("具体行动:\n")
            for action in sug['actions']:
                f.write(f"  {action}\n")
            f.write("\n")
    
    print(f"\n改进建议已保存到: {output_dir / 'improvement_suggestions.txt'}")
    
    return suggestions

def generate_improved_configs():
    """生成改进的PID配置建议"""
    print("\n" + "="*70)
    print("推荐的新PID配置")
    print("="*70)
    
    configs = [
        {
            'name': '极小增益组',
            'configs': [
                {'Kp': 0.05, 'Ki': 0.01, 'Kd': 0.02, 'desc': '弱约束，保持探索性'},
                {'Kp': 0.08, 'Ki': 0.015, 'Kd': 0.03, 'desc': '轻度引导'},
                {'Kp': 0.10, 'Ki': 0.02, 'Kd': 0.04, 'desc': '当前最小值'}
            ]
        },
        {
            'name': '强Ki组（加快收敛）',
            'configs': [
                {'Kp': 0.15, 'Ki': 0.05, 'Kd': 0.06, 'desc': 'Ki增大3倍'},
                {'Kp': 0.20, 'Ki': 0.08, 'Kd': 0.08, 'desc': 'Ki增大4倍'},
                {'Kp': 0.25, 'Ki': 0.10, 'Kd': 0.10, 'desc': 'Ki增大5倍'}
            ]
        },
        {
            'name': '渐进式PID（新策略）',
            'configs': [
                {'mode': 'progressive', 'desc': '前50%迭代: Kp×0.5, 后50%迭代: Kp×1.0'},
                {'mode': 'adaptive_strong', 'desc': '根据成功率动态调整: 成功率低→降低Kp'}
            ]
        }
    ]
    
    for group in configs:
        print(f"\n【{group['name']}】")
        for i, cfg in enumerate(group['configs'], 1):
            if 'mode' in cfg:
                print(f"  {i}. 模式: {cfg['mode']}")
                print(f"     说明: {cfg['desc']}")
            else:
                print(f"  {i}. Kp={cfg['Kp']}, Ki={cfg['Ki']}, Kd={cfg['Kd']}")
                print(f"     说明: {cfg['desc']}")
    
    # 保存配置
    output_dir = Path("results/diagnosis")
    output_dir.mkdir(exist_ok=True)
    
    import json
    with open(output_dir / 'improved_configs.json', 'w', encoding='utf-8') as f:
        json.dump(configs, f, indent=2, ensure_ascii=False)
    
    print(f"\n配置已保存到: {output_dir / 'improved_configs.json'}")

def main():
    """主函数"""
    print("="*70)
    print("PID实验问题诊断工具")
    print("="*70)
    
    # 加载数据
    df = load_latest_results()
    print(f"\n数据加载成功: {len(df)} 条记录")
    
    # 深度分析
    esr_df = analyze_problem(df)
    
    # 生成诊断图表
    generate_diagnostic_plots(df)
    
    # 生成改进建议
    suggestions = generate_improvement_suggestions(df)
    
    # 生成新配置建议
    generate_improved_configs()
    
    print("\n" + "="*70)
    print("诊断完成！")
    print("="*70)
    print("\n请查看:")
    print("  1. results/diagnosis/success_vs_esr.png - 成功率与ESR关系图")
    print("  2. results/diagnosis/efficiency_comparison.png - 效率对比图")
    print("  3. results/diagnosis/improvement_suggestions.txt - 改进建议")
    print("  4. results/diagnosis/improved_configs.json - 新配置方案")
    
if __name__ == '__main__':
    main()
