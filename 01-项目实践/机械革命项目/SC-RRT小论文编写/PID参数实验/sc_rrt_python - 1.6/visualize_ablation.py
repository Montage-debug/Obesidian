# -*- coding: utf-8 -*-
"""
消融实验结果可视化
Visualization for Ablation Study Results

生成SCI论文级别的图表
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import seaborn as sns
from scipy import stats

# 设置绘图样式
plt.style.use('seaborn-v0_8-paper')
sns.set_palette("husl")
plt.rcParams['font.family'] = 'Arial'
plt.rcParams['font.size'] = 10
plt.rcParams['figure.dpi'] = 300

class AblationVisualizer:
    """消融实验可视化器"""
    
    def __init__(self, results_csv):
        self.df = pd.read_csv(results_csv)
        self.output_dir = Path(results_csv).parent / 'figures'
        self.output_dir.mkdir(exist_ok=True)
        
        # 算法颜色映射
        self.colors = {
            'IRRT': '#E74C3C',        # 红色 - Informed-RRT*
            'SC-Static': '#F39C12',   # 橙色 - SC-RRT Static
            'SC-Adaptive': '#27AE60'  # 绿色 - SC-RRT Adaptive
        }
        
    def generate_all_figures(self):
        """生成所有图表"""
        print("生成消融实验可视化图表...")
        
        # 图1: 成功率对比
        self.plot_success_rate()
        
        # 图2: 收敛速度对比
        self.plot_convergence_speed()
        
        # 图3: 路径质量对比
        self.plot_path_quality()
        
        # 图4: 计算效率对比
        self.plot_computational_efficiency()
        
        # 图5: 综合雷达图
        self.plot_radar_chart()
        
        # 图6: 统计显著性检验
        self.plot_statistical_tests()
        
        print(f"所有图表已保存至: {self.output_dir}")
    
    def plot_success_rate(self):
        """绘制成功率对比图"""
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        
        for idx, scenario in enumerate(['2D_HighDensity', '3D_HighDensity']):
            ax = axes[idx]
            scenario_df = self.df[self.df['scenario'] == scenario]
            
            success_rates = []
            labels = []
            colors = []
            
            for algo in ['IRRT', 'SC-Static', 'SC-Adaptive']:
                algo_df = scenario_df[scenario_df['short_name'] == algo]
                sr = 100 * algo_df['success'].sum() / len(algo_df)
                success_rates.append(sr)
                labels.append(algo)
                colors.append(self.colors[algo])
            
            bars = ax.bar(labels, success_rates, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
            
            # 添加数值标签
            for bar in bars:
                height = bar.get_height()
                ax.text(bar.get_x() + bar.get_width()/2., height,
                       f'{height:.1f}%',
                       ha='center', va='bottom', fontsize=9, fontweight='bold')
            
            ax.set_ylabel('Success Rate (%)', fontsize=11, fontweight='bold')
            ax.set_title(f'{scenario.replace("_", " ")}', fontsize=12, fontweight='bold')
            ax.set_ylim([0, 100])
            ax.grid(axis='y', alpha=0.3, linestyle='--')
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
        
        plt.tight_layout()
        plt.savefig(self.output_dir / 'fig1_success_rate.png', dpi=300, bbox_inches='tight')
        plt.savefig(self.output_dir / 'fig1_success_rate.pdf', bbox_inches='tight')
        plt.close()
        print("  [√] 图1: 成功率对比")
    
    def plot_convergence_speed(self):
        """绘制收敛速度对比"""
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        
        for idx, scenario in enumerate(['2D_HighDensity', '3D_HighDensity']):
            ax = axes[idx]
            scenario_df = self.df[(self.df['scenario'] == scenario) & (self.df['success'] == True)]
            
            data_to_plot = []
            labels = []
            colors_list = []
            
            for algo in ['IRRT', 'SC-Static', 'SC-Adaptive']:
                algo_df = scenario_df[scenario_df['short_name'] == algo]
                if len(algo_df) > 0:
                    data_to_plot.append(algo_df['iterations'].values)
                    labels.append(algo)
                    colors_list.append(self.colors[algo])
            
            bp = ax.boxplot(data_to_plot, labels=labels, patch_artist=True,
                           showmeans=True, meanline=True,
                           boxprops=dict(linewidth=1.5),
                           whiskerprops=dict(linewidth=1.5),
                           capprops=dict(linewidth=1.5),
                           medianprops=dict(color='red', linewidth=2),
                           meanprops=dict(color='blue', linewidth=2, linestyle='--'))
            
            for patch, color in zip(bp['boxes'], colors_list):
                patch.set_facecolor(color)
                patch.set_alpha(0.6)
            
            ax.set_ylabel('Iterations to Success', fontsize=11, fontweight='bold')
            ax.set_title(f'{scenario.replace("_", " ")}', fontsize=12, fontweight='bold')
            ax.grid(axis='y', alpha=0.3, linestyle='--')
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
        
        plt.tight_layout()
        plt.savefig(self.output_dir / 'fig2_convergence_speed.png', dpi=300, bbox_inches='tight')
        plt.savefig(self.output_dir / 'fig2_convergence_speed.pdf', bbox_inches='tight')
        plt.close()
        print("  [√] 图2: 收敛速度对比")
    
    def plot_path_quality(self):
        """绘制路径质量对比"""
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        
        for idx, scenario in enumerate(['2D_HighDensity', '3D_HighDensity']):
            ax = axes[idx]
            scenario_df = self.df[(self.df['scenario'] == scenario) & (self.df['success'] == True)]
            
            data_to_plot = []
            labels = []
            colors_list = []
            
            for algo in ['IRRT', 'SC-Static', 'SC-Adaptive']:
                algo_df = scenario_df[scenario_df['short_name'] == algo]
                if len(algo_df) > 0:
                    data_to_plot.append(algo_df['path_length'].values)
                    labels.append(algo)
                    colors_list.append(self.colors[algo])
            
            bp = ax.boxplot(data_to_plot, labels=labels, patch_artist=True,
                           showmeans=True, meanline=True,
                           boxprops=dict(linewidth=1.5),
                           whiskerprops=dict(linewidth=1.5),
                           capprops=dict(linewidth=1.5),
                           medianprops=dict(color='red', linewidth=2),
                           meanprops=dict(color='blue', linewidth=2, linestyle='--'))
            
            for patch, color in zip(bp['boxes'], colors_list):
                patch.set_facecolor(color)
                patch.set_alpha(0.6)
            
            ax.set_ylabel('Path Length', fontsize=11, fontweight='bold')
            ax.set_title(f'{scenario.replace("_", " ")}', fontsize=12, fontweight='bold')
            ax.grid(axis='y', alpha=0.3, linestyle='--')
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
        
        plt.tight_layout()
        plt.savefig(self.output_dir / 'fig3_path_quality.png', dpi=300, bbox_inches='tight')
        plt.savefig(self.output_dir / 'fig3_path_quality.pdf', bbox_inches='tight')
        plt.close()
        print("  [√] 图3: 路径质量对比")
    
    def plot_computational_efficiency(self):
        """绘制计算效率对比"""
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        
        for idx, scenario in enumerate(['2D_HighDensity', '3D_HighDensity']):
            ax = axes[idx]
            scenario_df = self.df[(self.df['scenario'] == scenario) & (self.df['success'] == True)]
            
            algos = ['IRRT', 'SC-Static', 'SC-Adaptive']
            means = []
            stds = []
            colors_list = []
            
            for algo in algos:
                algo_df = scenario_df[scenario_df['short_name'] == algo]
                if len(algo_df) > 0:
                    means.append(algo_df['planning_time'].mean())
                    stds.append(algo_df['planning_time'].std())
                    colors_list.append(self.colors[algo])
                else:
                    means.append(0)
                    stds.append(0)
                    colors_list.append(self.colors[algo])
            
            x = np.arange(len(algos))
            bars = ax.bar(x, means, yerr=stds, color=colors_list, alpha=0.8,
                         edgecolor='black', linewidth=1.5, capsize=5, error_kw={'linewidth': 2})
            
            # 添加数值标签
            for i, (mean, std) in enumerate(zip(means, stds)):
                if mean > 0:
                    ax.text(i, mean + std, f'{mean:.2f}s',
                           ha='center', va='bottom', fontsize=9, fontweight='bold')
            
            ax.set_xticks(x)
            ax.set_xticklabels(algos)
            ax.set_ylabel('Planning Time (s)', fontsize=11, fontweight='bold')
            ax.set_title(f'{scenario.replace("_", " ")}', fontsize=12, fontweight='bold')
            ax.grid(axis='y', alpha=0.3, linestyle='--')
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
        
        plt.tight_layout()
        plt.savefig(self.output_dir / 'fig4_computational_efficiency.png', dpi=300, bbox_inches='tight')
        plt.savefig(self.output_dir / 'fig4_computational_efficiency.pdf', bbox_inches='tight')
        plt.close()
        print("  [√] 图4: 计算效率对比")
    
    def plot_radar_chart(self):
        """绘制综合性能雷达图"""
        from math import pi
        
        fig, axes = plt.subplots(1, 2, figsize=(12, 5), subplot_kw=dict(projection='polar'))
        
        # 指标
        categories = ['Success\nRate', 'Convergence\nSpeed', 'Path\nQuality', 
                     'Sampling\nEfficiency', 'Computational\nEfficiency']
        N = len(categories)
        
        for idx, scenario in enumerate(['2D_HighDensity', '3D_HighDensity']):
            ax = axes[idx]
            scenario_df = self.df[(self.df['scenario'] == scenario) & (self.df['success'] == True)]
            
            angles = [n / float(N) * 2 * pi for n in range(N)]
            angles += angles[:1]
            
            ax.set_theta_offset(pi / 2)
            ax.set_theta_direction(-1)
            ax.set_xticks(angles[:-1])
            ax.set_xticklabels(categories, fontsize=9)
            
            for algo in ['IRRT', 'SC-Static', 'SC-Adaptive']:
                algo_df = scenario_df[scenario_df['short_name'] == algo]
                
                if len(algo_df) > 0:
                    # 归一化指标 (0-1)
                    all_success = self.df[self.df['scenario'] == scenario]
                    sr = algo_df['success'].sum() / len(self.df[self.df['scenario'] == scenario])
                    
                    # 收敛速度（迭代次数越少越好，取倒数归一化）
                    max_iter = scenario_df['iterations'].max()
                    cs = 1 - (algo_df['iterations'].mean() / max_iter)
                    
                    # 路径质量（路径越短越好，取倒数归一化）
                    max_path = scenario_df['path_length'].max()
                    pq = 1 - (algo_df['path_length'].mean() / max_path)
                    
                    # 采样效率
                    se = algo_df['effective_sampling_ratio'].mean()
                    
                    # 计算效率（时间越短越好）
                    max_time = scenario_df['planning_time'].max()
                    ce = 1 - (algo_df['planning_time'].mean() / max_time)
                    
                    values = [sr, cs, pq, se, ce]
                    values += values[:1]
                    
                    ax.plot(angles, values, 'o-', linewidth=2, label=algo, 
                           color=self.colors[algo])
                    ax.fill(angles, values, alpha=0.15, color=self.colors[algo])
            
            ax.set_ylim([0, 1])
            ax.set_title(f'{scenario.replace("_", " ")}', fontsize=12, fontweight='bold', pad=20)
            ax.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1), fontsize=9)
            ax.grid(True, linestyle='--', alpha=0.5)
        
        plt.tight_layout()
        plt.savefig(self.output_dir / 'fig5_radar_chart.png', dpi=300, bbox_inches='tight')
        plt.savefig(self.output_dir / 'fig5_radar_chart.pdf', bbox_inches='tight')
        plt.close()
        print("  [√] 图5: 综合性能雷达图")
    
    def plot_statistical_tests(self):
        """绘制统计显著性检验结果"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        scenarios = ['2D_HighDensity', '3D_HighDensity']
        metrics = [('iterations', 'Iterations'), ('path_length', 'Path Length')]
        
        for s_idx, scenario in enumerate(scenarios):
            for m_idx, (metric, label) in enumerate(metrics):
                ax = axes[s_idx, m_idx]
                scenario_df = self.df[(self.df['scenario'] == scenario) & (self.df['success'] == True)]
                
                # 准备数据
                data_dict = {}
                for algo in ['IRRT', 'SC-Static', 'SC-Adaptive']:
                    algo_df = scenario_df[scenario_df['short_name'] == algo]
                    if len(algo_df) > 0:
                        data_dict[algo] = algo_df[metric].values
                
                if len(data_dict) >= 2:
                    # t检验
                    algos = list(data_dict.keys())
                    n_algos = len(algos)
                    p_values = np.ones((n_algos, n_algos))
                    
                    for i in range(n_algos):
                        for j in range(i+1, n_algos):
                            _, p = stats.ttest_ind(data_dict[algos[i]], data_dict[algos[j]])
                            p_values[i, j] = p
                            p_values[j, i] = p
                    
                    # 绘制热力图
                    im = ax.imshow(p_values, cmap='RdYlGn_r', vmin=0, vmax=0.1)
                    ax.set_xticks(np.arange(n_algos))
                    ax.set_yticks(np.arange(n_algos))
                    ax.set_xticklabels(algos)
                    ax.set_yticklabels(algos)
                    
                    # 添加p值文本
                    for i in range(n_algos):
                        for j in range(n_algos):
                            if i != j:
                                sig = '***' if p_values[i, j] < 0.001 else \
                                      '**' if p_values[i, j] < 0.01 else \
                                      '*' if p_values[i, j] < 0.05 else 'n.s.'
                                text = ax.text(j, i, f'{p_values[i, j]:.3f}\n{sig}',
                                             ha="center", va="center", color="black", fontsize=8)
                    
                    ax.set_title(f'{scenario.replace("_", " ")} - {label}', 
                               fontsize=11, fontweight='bold')
                    
                    # 添加颜色条
                    plt.colorbar(im, ax=ax, label='p-value', fraction=0.046, pad=0.04)
        
        plt.tight_layout()
        plt.savefig(self.output_dir / 'fig6_statistical_tests.png', dpi=300, bbox_inches='tight')
        plt.savefig(self.output_dir / 'fig6_statistical_tests.pdf', bbox_inches='tight')
        plt.close()
        print("  [√] 图6: 统计显著性检验")


def main():
    import sys
    
    if len(sys.argv) < 2:
        print("用法: python visualize_ablation.py <results_csv_file>")
        print("示例: python visualize_ablation.py results/ablation/ablation_results_20260216_140814.csv")
        return
    
    results_file = sys.argv[1]
    
    if not Path(results_file).exists():
        print(f"错误: 文件不存在: {results_file}")
        return
    
    visualizer = AblationVisualizer(results_file)
    visualizer.generate_all_figures()
    print("\n可视化完成！")


if __name__ == '__main__':
    main()
