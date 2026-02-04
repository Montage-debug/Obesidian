"""
对比改进前后的实验结果
Compare Experiment Results: Before vs After Improvement
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

def load_results(csv_path):
    """加载实验结果"""
    df = pd.read_csv(csv_path)
    return df

def compare_success_rates(df_old, df_new):
    """对比成功率"""
    print("\n" + "="*70)
    print("【1】成功率对比")
    print("="*70)
    
    # 计算成功率
    old_success = df_old['success'].mean() * 100
    new_success = df_new['success'].mean() * 100
    improvement = new_success - old_success
    
    print(f"\n改进前: {old_success:.1f}%  ({df_old['success'].sum()}/{len(df_old)})")
    print(f"改进后: {new_success:.1f}%  ({df_new['success'].sum()}/{len(df_new)})")
    print(f"提升:   {improvement:+.1f}%  ({improvement/old_success*100:+.1f}%相对提升)")
    
    # 按PID配置分组
    print("\n按PID配置分组:")
    old_grouped = df_old.groupby('pid_config')['success'].mean() * 100
    new_grouped = df_new.groupby('pid_config')['success'].mean() * 100
    
    comparison = pd.DataFrame({
        '改进前(%)': old_grouped,
        '改进后(%)': new_grouped,
        '提升(%)': new_grouped - old_grouped
    }).round(1)
    print(comparison.to_string())
    
    return comparison

def compare_esr(df_old, df_new):
    """对比有效采样比例"""
    print("\n" + "="*70)
    print("【2】有效采样比例(ESR)对比")
    print("="*70)
    
    # 计算ESR
    old_esr = df_old['esr'].mean() * 100
    new_esr = df_new['esr'].mean() * 100
    improvement = new_esr - old_esr
    
    print(f"\n改进前: {old_esr:.1f}%")
    print(f"改进后: {new_esr:.1f}%")
    print(f"提升:   {improvement:+.1f}%")
    
    # 按PID配置分组
    print("\n按PID配置分组:")
    old_grouped = df_old.groupby('pid_config')['esr'].mean() * 100
    new_grouped = df_new.groupby('pid_config')['esr'].mean() * 100
    
    comparison = pd.DataFrame({
        '改进前(%)': old_grouped,
        '改进后(%)': new_grouped,
        '提升(%)': new_grouped - old_grouped
    }).round(1)
    print(comparison.to_string())
    
    return comparison

def compare_sample_to_success(df_old, df_new):
    """对比Sample-to-Success"""
    print("\n" + "="*70)
    print("【3】Sample-to-Success对比（成功样本）")
    print("="*70)
    
    # 只看成功样本
    old_success = df_old[df_old['success'] == True]
    new_success = df_new[df_new['success'] == True]
    
    if len(old_success) == 0 or len(new_success) == 0:
        print("警告：没有足够的成功样本进行对比")
        return None
    
    old_samples = old_success['samples_to_success'].mean()
    new_samples = new_success['samples_to_success'].mean()
    improvement = old_samples - new_samples
    
    print(f"\n改进前: {old_samples:.1f} 次")
    print(f"改进后: {new_samples:.1f} 次")
    print(f"减少:   {improvement:.1f} 次 ({improvement/old_samples*100:.1f}%)")
    
    # 按PID配置分组
    print("\n按PID配置分组:")
    old_grouped = old_success.groupby('pid_config')['samples_to_success'].agg(['mean', 'std'])
    new_grouped = new_success.groupby('pid_config')['samples_to_success'].agg(['mean', 'std'])
    
    comparison = pd.DataFrame({
        '改进前_均值': old_grouped['mean'],
        '改进前_标准差': old_grouped['std'],
        '改进后_均值': new_grouped['mean'],
        '改进后_标准差': new_grouped['std'],
        '减少量': old_grouped['mean'] - new_grouped['mean']
    }).round(1)
    print(comparison.to_string())
    
    return comparison

def compare_path_quality(df_old, df_new):
    """对比路径质量"""
    print("\n" + "="*70)
    print("【4】路径质量对比（成功样本）")
    print("="*70)
    
    # 只看成功样本
    old_success = df_old[df_old['success'] == True]
    new_success = df_new[df_new['success'] == True]
    
    if len(old_success) == 0 or len(new_success) == 0:
        print("警告：没有足够的成功样本进行对比")
        return None
    
    # 路径长度
    old_length = old_success['path_length'].mean()
    new_length = new_success['path_length'].mean()
    
    # 路径效率
    old_efficiency = old_success['path_efficiency'].mean()
    new_efficiency = new_success['path_efficiency'].mean()
    
    print(f"\n路径长度:")
    print(f"  改进前: {old_length:.1f}")
    print(f"  改进后: {new_length:.1f}")
    print(f"  变化:   {new_length-old_length:+.1f} ({(new_length-old_length)/old_length*100:+.1f}%)")
    
    print(f"\n路径效率:")
    print(f"  改进前: {old_efficiency:.2f}")
    print(f"  改进后: {new_efficiency:.2f}")
    print(f"  变化:   {new_efficiency-old_efficiency:+.2f}")

def visualize_comparison(df_old, df_new, output_dir):
    """生成对比可视化"""
    print("\n" + "="*70)
    print("【5】生成对比可视化图表")
    print("="*70)
    
    output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)
    
    # 1. 成功率对比
    fig, ax = plt.subplots(figsize=(10, 6))
    
    configs = sorted(df_old['pid_config'].unique())
    old_rates = [df_old[df_old['pid_config']==c]['success'].mean()*100 for c in configs]
    new_rates = [df_new[df_new['pid_config']==c]['success'].mean()*100 for c in configs]
    
    x = np.arange(len(configs))
    width = 0.35
    
    ax.bar(x - width/2, old_rates, width, label='改进前', alpha=0.8, color='skyblue')
    ax.bar(x + width/2, new_rates, width, label='改进后', alpha=0.8, color='coral')
    
    ax.set_xlabel('PID配置', fontsize=12, fontproperties='SimHei')
    ax.set_ylabel('成功率 (%)', fontsize=12, fontproperties='SimHei')
    ax.set_title('改进前后成功率对比', fontsize=14, fontproperties='SimHei')
    ax.set_xticks(x)
    ax.set_xticklabels(configs, rotation=45, ha='right', fontproperties='SimHei')
    ax.legend(prop={'family': 'SimHei'})
    ax.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'comparison_success_rate.png', dpi=300, bbox_inches='tight')
    print(f"✓ 成功率对比图: {output_dir / 'comparison_success_rate.png'}")
    plt.close()
    
    # 2. ESR对比
    fig, ax = plt.subplots(figsize=(10, 6))
    
    old_esr = [df_old[df_old['pid_config']==c]['esr'].mean()*100 for c in configs]
    new_esr = [df_new[df_new['pid_config']==c]['esr'].mean()*100 for c in configs]
    
    ax.bar(x - width/2, old_esr, width, label='改进前', alpha=0.8, color='lightgreen')
    ax.bar(x + width/2, new_esr, width, label='改进后', alpha=0.8, color='orange')
    
    ax.set_xlabel('PID配置', fontsize=12, fontproperties='SimHei')
    ax.set_ylabel('有效采样比例 (%)', fontsize=12, fontproperties='SimHei')
    ax.set_title('改进前后ESR对比', fontsize=14, fontproperties='SimHei')
    ax.set_xticks(x)
    ax.set_xticklabels(configs, rotation=45, ha='right', fontproperties='SimHei')
    ax.legend(prop={'family': 'SimHei'})
    ax.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'comparison_esr.png', dpi=300, bbox_inches='tight')
    print(f"✓ ESR对比图: {output_dir / 'comparison_esr.png'}")
    plt.close()

def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='对比改进前后的实验结果')
    parser.add_argument('--old', type=str, required=True, help='改进前的结果文件')
    parser.add_argument('--new', type=str, required=True, help='改进后的结果文件')
    parser.add_argument('--output', type=str, default='comparison_results', help='输出目录')
    
    args = parser.parse_args()
    
    print("="*70)
    print("SC-RRT PID实验改进效果对比分析")
    print("="*70)
    print(f"\n改进前数据: {args.old}")
    print(f"改进后数据: {args.new}")
    print(f"输出目录:   {args.output}")
    
    # 加载数据
    try:
        df_old = load_results(args.old)
        df_new = load_results(args.new)
    except Exception as e:
        print(f"\n错误：无法加载数据文件")
        print(f"详情: {e}")
        sys.exit(1)
    
    print(f"\n改进前样本数: {len(df_old)}")
    print(f"改进后样本数: {len(df_new)}")
    
    # 执行对比分析
    compare_success_rates(df_old, df_new)
    compare_esr(df_old, df_new)
    compare_sample_to_success(df_old, df_new)
    compare_path_quality(df_old, df_new)
    
    # 生成可视化
    visualize_comparison(df_old, df_new, args.output)
    
    print("\n" + "="*70)
    print("对比分析完成！")
    print(f"结果已保存到: {args.output}/")
    print("="*70)

if __name__ == "__main__":
    main()
