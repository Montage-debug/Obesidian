"""
实验结果分析和可视化工具
Analysis and Visualization Tools for Experiment Results
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端，避免GUI问题
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
from typing import Optional
import warnings
warnings.filterwarnings('ignore')

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False


class ResultAnalyzer:
    """结果分析器"""
    
    def __init__(self, csv_path: str):
        """
        初始化分析器
        
        Args:
            csv_path: CSV结果文件路径
        """
        self.csv_path = Path(csv_path)
        self.df = pd.read_csv(csv_path)
        self.output_dir = self.csv_path.parent
        
        print(f"加载数据: {csv_path}")
        print(f"总记录数: {len(self.df)}")
        print(f"场景数: {self.df['scenario'].nunique()}")
        print(f"PID配置数: {self.df['pid_config'].nunique()}")
    
    def generate_summary_statistics(self):
        """生成汇总统计（三层评价体系 + ESR和Sample-to-Success核心指标）"""
        print("\n" + "="*70)
        print("生成汇总统计（三层评价体系 + 采样效率指标）")
        print("="*70)
        
        # 第一层：全样本统计（成功率）
        all_stats = self.df.groupby(['pid_config', 'Kp', 'Ki', 'Kd']).agg({
            'success': ['mean', 'sum', 'count']
        }).round(4)
        all_stats.columns = ['success_rate', 'success_count', 'total_count']
        all_stats = all_stats.reset_index()
        all_stats['success_rate_pct'] = all_stats['success_rate'] * 100
        
        # 第二层：成功样本统计（路径质量）
        df_success = self.df[self.df['success'] == True].copy()
        
        if len(df_success) > 0:
            # 计算单位迭代路径增长率（行为级指标）
            df_success['path_efficiency'] = df_success['path_length'] / df_success['tree_nodes']
            
            success_stats = df_success.groupby(['pid_config']).agg({
                'path_length': ['mean', 'std', 'min', 'max'],
                'tree_nodes': ['mean', 'std'],
                'path_efficiency': ['mean', 'std'],  # 新增行为级指标
                'planning_time': ['mean', 'std'],
                'smoothness': ['mean', 'std'],
                'convergence_time': ['mean', 'std']
            }).round(4)
            success_stats.columns = ['_'.join(col).strip() for col in success_stats.columns.values]
            success_stats = success_stats.reset_index()
            
            # 合并
            grouped = all_stats.merge(success_stats, on='pid_config', how='left')
        else:
            grouped = all_stats
            print("警告: 没有成功样本，无法计算路径质量指标")
        
        # 保存完整统计
        output_path = self.output_dir / 'summary_statistics.csv'
        grouped.to_csv(output_path, index=False, encoding='utf-8-sig')
        print(f"汇总统计已保存: {output_path}")
        
        # 打印全部配置的成功率
        print("\n所有PID配置的成功率:")
        print(grouped[['pid_config', 'Kp', 'Ki', 'Kd', 'success_rate_pct', 
                      'success_count', 'total_count']].to_string(index=False))
        
        # 如果有成功样本，显示路径质量
        if len(df_success) > 0:
            print("\n成功样本的路径质量指标:")
            quality_cols = ['pid_config', 'path_length_mean', 'path_length_std', 
                          'path_efficiency_mean', 'tree_nodes_mean', 'planning_time_mean']
            print(grouped[quality_cols].dropna().to_string(index=False))
        
        # ★★★ 新增：有效采样比例(ESR)统计 ★★★
        if 'effective_sampling_ratio' in self.df.columns:
            print("\n" + "="*70)
            print("【核心指标1】有效采样比例(ESR) - 证明PID引导采样进入约束区域")
            print("="*70)
            esr_stats = self.df.groupby('pid_config').agg({
                'effective_sampling_ratio': ['mean', 'std', 'min', 'max'],
                'samples_in_ellipsoid': 'sum',
                'total_samples_attempted': 'sum'
            }).round(4)
            esr_stats.columns = ['esr_mean', 'esr_std', 'esr_min', 'esr_max',
                                'ellipsoid_samples_total', 'total_samples']
            esr_stats = esr_stats.reset_index()
            esr_stats['esr_mean_pct'] = (esr_stats['esr_mean'] * 100).round(2)
            esr_stats['esr_std_pct'] = (esr_stats['esr_std'] * 100).round(2)
            
            print("\nESR统计（所有样本）:")
            print(esr_stats[['pid_config', 'esr_mean_pct', 'esr_std_pct', 
                            'ellipsoid_samples_total', 'total_samples']].to_string(index=False))
        
        # ★★★ 新增：Sample-to-Success统计 ★★★
        if 'first_solution_iter' in self.df.columns and len(df_success) > 0:
            print("\n" + "="*70)
            print("【核心指标2】Sample-to-Success - 证明PID提高单位采样价值")
            print("="*70)
            sts_stats = df_success.groupby('pid_config').agg({
                'first_solution_iter': ['mean', 'std', 'median', 'min', 'max']
            }).round(2)
            sts_stats.columns = ['samples_to_success_mean', 'samples_to_success_std',
                                'samples_to_success_median', 'samples_to_success_min',
                                'samples_to_success_max']
            sts_stats = sts_stats.reset_index()
            
            print("\nSample-to-Success统计（成功样本）:")
            print(sts_stats.to_string(index=False))
        
        # 打印核心对比
        if 'effective_sampling_ratio' in self.df.columns or 'first_solution_iter' in self.df.columns:
            print("\n" + "="*70)
            print("【关键指标对比】PID vs No_PID")
            print("="*70)
            comparison = grouped[['pid_config', 'success_rate_pct']].copy()
            
            if 'effective_sampling_ratio' in self.df.columns:
                esr_mean = self.df.groupby('pid_config')['effective_sampling_ratio'].mean().reset_index()
                esr_mean.columns = ['pid_config', 'esr_mean']
                comparison = comparison.merge(esr_mean, on='pid_config', how='left')
            
            if 'first_solution_iter' in self.df.columns and len(df_success) > 0:
                sts_mean = df_success.groupby('pid_config')['first_solution_iter'].mean().reset_index()
                sts_mean.columns = ['pid_config', 'samples_to_success']
                comparison = comparison.merge(sts_mean, on='pid_config', how='left')
            
            print(comparison.to_string(index=False))
        
        return grouped
    
    def find_optimal_pid(self, success_threshold=0.70):
        """
        分层最优判定逻辑
        
        Args:
            success_threshold: 成功率阈值（默认70%）
        
        Returns:
            最优PID配置及分析报告
        """
        print("\n" + "="*70)
        print(f"三层最优判定（成功率阈值≥{success_threshold*100:.0f}%）")
        print("="*70)
        
        # 第一层筛选：成功率
        candidates = self.df.groupby(['pid_config', 'Kp', 'Ki', 'Kd']).agg({
            'success': 'mean'
        }).reset_index()
        candidates.columns = ['pid_config', 'Kp', 'Ki', 'Kd', 'success_rate']
        candidates['success_rate_pct'] = candidates['success_rate'] * 100
        
        # 筛选候选集
        qualified = candidates[candidates['success_rate'] >= success_threshold].copy()
        
        if len(qualified) == 0:
            print(f"⚠ 没有配置达到成功率阈值{success_threshold*100:.0f}%")
            print("降低阈值重新筛选...")
            success_threshold = candidates['success_rate'].quantile(0.5)
            qualified = candidates[candidates['success_rate'] >= success_threshold].copy()
            print(f"新阈值: {success_threshold*100:.1f}%")
        
        print(f"\n✓ 第一层筛选: {len(qualified)}/{len(candidates)} 个配置通过成功率筛选")
        print(qualified[['pid_config', 'Kp', 'Ki', 'Kd', 'success_rate_pct']].to_string(index=False))
        
        # 第二层比较：路径质量（只在成功样本上）
        df_success = self.df[self.df['success'] == True].copy()
        
        if len(df_success) == 0:
            print("\n⚠ 没有成功样本，无法进行第二层比较")
            return candidates.sort_values('success_rate', ascending=False).iloc[0]
        
        # 计算行为级指标
        df_success['path_efficiency'] = df_success['path_length'] / df_success['tree_nodes']
        
        # 只对候选集进行质量分析
        quality = df_success[df_success['pid_config'].isin(qualified['pid_config'])].groupby('pid_config').agg({
            'path_length': ['mean', 'std'],
            'path_efficiency': ['mean', 'std'],
            'tree_nodes': ['mean', 'std']
        }).round(4)
        quality.columns = ['_'.join(col) for col in quality.columns]
        quality = quality.reset_index()
        
        # 合并成功率和质量指标
        final = qualified.merge(quality, on='pid_config', how='left')
        
        print(f"\n✓ 第二层比较: 成功样本的路径质量")
        print(final[['pid_config', 'success_rate_pct', 'path_length_mean', 
                    'path_efficiency_mean', 'tree_nodes_mean']].to_string(index=False))
        
        # 第三层优化：综合排序
        # 排序策略：成功率 > 路径长度 > 路径效率 > 树节点数
        final_sorted = final.sort_values(
            ['success_rate', 'path_length_mean', 'path_efficiency_mean', 'tree_nodes_mean'],
            ascending=[False, True, False, True]
        )
        
        optimal = final_sorted.iloc[0]
        
        print(f"\n" + "="*70)
        print("🏆 最优PID配置")
        print("="*70)
        print(f"配置名称: {optimal['pid_config']}")
        print(f"参数: Kp={optimal['Kp']:.2f}, Ki={optimal['Ki']:.2f}, Kd={optimal['Kd']:.2f}")
        print(f"\n性能指标:")
        print(f"  • 成功率: {optimal['success_rate_pct']:.1f}%")
        print(f"  • 平均路径长度: {optimal['path_length_mean']:.2f} ± {optimal['path_length_std']:.2f}")
        print(f"  • 单位迭代路径效率: {optimal['path_efficiency_mean']:.4f} ± {optimal['path_efficiency_std']:.4f}")
        print(f"  • 平均树节点数: {optimal['tree_nodes_mean']:.1f} ± {optimal['tree_nodes_std']:.1f}")
        print("="*70)
        
        # 保存最优结果
        output_path = self.output_dir / 'optimal_pid_result.txt'
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write("="*70 + "\n")
            f.write("最优PID配置分析报告\n")
            f.write("="*70 + "\n\n")
            f.write(f"配置名称: {optimal['pid_config']}\n")
            f.write(f"参数: Kp={optimal['Kp']:.2f}, Ki={optimal['Ki']:.2f}, Kd={optimal['Kd']:.2f}\n\n")
            f.write(f"性能指标:\n")
            f.write(f"  成功率: {optimal['success_rate_pct']:.1f}%\n")
            f.write(f"  平均路径长度: {optimal['path_length_mean']:.2f} ± {optimal['path_length_std']:.2f}\n")
            f.write(f"  单位迭代路径效率: {optimal['path_efficiency_mean']:.4f} ± {optimal['path_efficiency_std']:.4f}\n")
            f.write(f"  平均树节点数: {optimal['tree_nodes_mean']:.1f} ± {optimal['tree_nodes_std']:.1f}\n")
        
        print(f"\n最优配置报告已保存: {output_path}")
        
        return optimal
    
    def plot_success_rate_comparison(self):
        """绘制成功率对比图"""
        print("\n生成成功率对比图...")
        
        # 按PID配置计算成功率
        success_rate = self.df.groupby('pid_config')['success'].mean().sort_values(ascending=False)
        
        fig, ax = plt.subplots(figsize=(14, 6))
        bars = ax.bar(range(len(success_rate)), success_rate.values * 100)
        
        # 颜色映射
        colors = plt.cm.RdYlGn(success_rate.values)
        for bar, color in zip(bars, colors):
            bar.set_color(color)
        
        ax.set_xlabel('PID配置', fontsize=12)
        ax.set_ylabel('成功率 (%)', fontsize=12)
        ax.set_title('不同PID参数配置的成功率对比', fontsize=14, fontweight='bold')
        ax.set_xticks(range(len(success_rate)))
        ax.set_xticklabels(success_rate.index, rotation=45, ha='right', fontsize=8)
        ax.grid(axis='y', alpha=0.3)
        ax.set_ylim([0, 105])
        
        plt.tight_layout()
        output_path = self.output_dir / 'success_rate_comparison.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"已保存: {output_path}")
    
    def plot_path_length_comparison(self):
        """绘制路径长度对比图"""
        print("\n生成路径长度对比图...")
        
        # 只统计成功的案例
        df_success = self.df[self.df['success'] == True].copy()
        
        if len(df_success) == 0:
            print("警告: 没有成功的案例，跳过路径长度对比图")
            return
        
        # 按PID配置分组
        grouped = df_success.groupby('pid_config')['path_length'].agg(['mean', 'std']).sort_values('mean')
        
        fig, ax = plt.subplots(figsize=(14, 6))
        x = range(len(grouped))
        ax.bar(x, grouped['mean'], yerr=grouped['std'], capsize=3, alpha=0.7, color='steelblue')
        
        ax.set_xlabel('PID配置', fontsize=12)
        ax.set_ylabel('平均路径长度', fontsize=12)
        ax.set_title('不同PID参数配置的路径长度对比（仅成功案例）', fontsize=14, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(grouped.index, rotation=45, ha='right', fontsize=8)
        ax.grid(axis='y', alpha=0.3)
        
        plt.tight_layout()
        output_path = self.output_dir / 'path_length_comparison.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"已保存: {output_path}")
    
    def plot_parameter_sensitivity(self):
        """绘制参数敏感度分析热力图"""
        print("\n生成参数敏感度热力图...")
        
        # 创建参数网格
        metrics = ['success', 'path_length', 'planning_time', 'smoothness']
        param_names = ['Kp', 'Ki', 'Kd']
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        axes = axes.flatten()
        
        for idx, metric in enumerate(metrics):
            ax = axes[idx]
            
            # 对于成功率，使用全部数据；对于其他指标，只用成功案例
            if metric == 'success':
                data = self.df
            else:
                data = self.df[self.df['success'] == True]
            
            if len(data) == 0:
                ax.text(0.5, 0.5, f'无数据', ha='center', va='center', fontsize=12)
                ax.set_title(f'{metric} 敏感度', fontsize=12, fontweight='bold')
                continue
            
            # 计算相关性 - 修复形状不一致问题
            corr_data = []
            max_len = 0
            for param in param_names:
                if metric == 'success':
                    corr = data.groupby(param)[metric].mean()
                else:
                    corr = data.groupby(param)[metric].mean()
                corr_data.append(corr.values)
                max_len = max(max_len, len(corr.values))
            
            # 填充到相同长度
            corr_data_padded = []
            for corr_values in corr_data:
                if len(corr_values) < max_len:
                    # 用NaN填充
                    padded = np.full(max_len, np.nan)
                    padded[:len(corr_values)] = corr_values
                    corr_data_padded.append(padded)
                else:
                    corr_data_padded.append(corr_values)
            
            # 归一化到0-1
            corr_matrix = np.array(corr_data_padded)
            
            # 绘制
            im = ax.imshow(corr_matrix, cmap='YlOrRd', aspect='auto')
            
            ax.set_yticks(range(len(param_names)))
            ax.set_yticklabels(param_names)
            ax.set_xlabel('参数值索引', fontsize=10)
            ax.set_title(f'{metric} 敏感度', fontsize=12, fontweight='bold')
            
            # 添加colorbar
            plt.colorbar(im, ax=ax)
        
        plt.tight_layout()
        output_path = self.output_dir / 'parameter_sensitivity.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"已保存: {output_path}")
    
    def plot_boxplot_by_scenario(self):
        """绘制不同场景下的性能箱线图"""
        print("\n生成场景性能箱线图...")
        
        df_success = self.df[self.df['success'] == True].copy()
        
        if len(df_success) == 0:
            print("警告: 没有成功的案例，跳过箱线图")
            return
        
        fig, axes = plt.subplots(1, 3, figsize=(16, 5))
        
        metrics = [
            ('path_length', '路径长度'),
            ('planning_time', '规划时间 (s)'),
            ('smoothness', '平滑度 (度)')
        ]
        
        for idx, (metric, label) in enumerate(metrics):
            ax = axes[idx]
            
            scenarios = df_success['scenario'].unique()
            data_to_plot = [df_success[df_success['scenario'] == s][metric].values 
                           for s in scenarios]
            
            bp = ax.boxplot(data_to_plot, labels=scenarios, patch_artist=True)
            
            # 美化
            for patch in bp['boxes']:
                patch.set_facecolor('lightblue')
                patch.set_alpha(0.7)
            
            ax.set_ylabel(label, fontsize=11)
            ax.set_xlabel('场景', fontsize=11)
            ax.set_title(f'{label}分布', fontsize=12, fontweight='bold')
            ax.grid(axis='y', alpha=0.3)
        
        plt.tight_layout()
        output_path = self.output_dir / 'performance_boxplot_by_scenario.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"已保存: {output_path}")
    
    def plot_pareto_frontier(self):
        """绘制Pareto前沿（路径长度 vs 规划时间）"""
        print("\n生成Pareto前沿图...")
        
        df_success = self.df[self.df['success'] == True].copy()
        
        if len(df_success) == 0:
            print("警告: 没有成功的案例，跳过Pareto前沿图")
            return
        
        # 按PID配置分组取平均
        grouped = df_success.groupby('pid_config').agg({
            'path_length': 'mean',
            'planning_time': 'mean',
            'Kp': 'first',
            'Ki': 'first',
            'Kd': 'first'
        }).reset_index()
        
        fig, ax = plt.subplots(figsize=(10, 6))
        
        scatter = ax.scatter(
            grouped['planning_time'],
            grouped['path_length'],
            c=grouped['Kp'],
            s=100,
            alpha=0.6,
            cmap='viridis',
            edgecolors='black'
        )
        
        ax.set_xlabel('平均规划时间 (s)', fontsize=12)
        ax.set_ylabel('平均路径长度', fontsize=12)
        ax.set_title('Pareto前沿: 路径长度 vs 规划时间', fontsize=14, fontweight='bold')
        ax.grid(alpha=0.3)
        
        # 添加colorbar
        cbar = plt.colorbar(scatter, ax=ax)
        cbar.set_label('Kp 值', fontsize=11)
        
        # 标注最优点
        best_idx = grouped['path_length'].idxmin()
        ax.annotate(
            f"最优: {grouped.loc[best_idx, 'pid_config']}",
            xy=(grouped.loc[best_idx, 'planning_time'], grouped.loc[best_idx, 'path_length']),
            xytext=(10, 10),
            textcoords='offset points',
            fontsize=9,
            bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.7),
            arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0')
        )
        
        plt.tight_layout()
        output_path = self.output_dir / 'pareto_frontier.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"已保存: {output_path}")
    
    def plot_path_efficiency_comparison(self):
        """绘制路径效率对比图（行为级指标）"""
        print("\n生成路径效率对比图...")
        
        df_success = self.df[self.df['success'] == True].copy()
        
        if len(df_success) == 0:
            print("警告: 没有成功的案例，跳过路径效率对比图")
            return
        
        # 计算路径效率
        df_success['path_efficiency'] = df_success['path_length'] / df_success['tree_nodes']
        
        # 按PID配置分组
        grouped = df_success.groupby('pid_config')['path_efficiency'].agg(['mean', 'std']).sort_values('mean', ascending=False)
        
        fig, ax = plt.subplots(figsize=(14, 6))
        x = range(len(grouped))
        bars = ax.bar(x, grouped['mean'], yerr=grouped['std'], capsize=3, alpha=0.7)
        
        # 根据效率高低着色
        colors = plt.cm.RdYlGn(grouped['mean'] / grouped['mean'].max())
        for bar, color in zip(bars, colors):
            bar.set_color(color)
        
        ax.set_xlabel('PID配置', fontsize=12)
        ax.set_ylabel('单位迭代路径增长率 (路径长度/迭代次数)', fontsize=12)
        ax.set_title('不同PID参数的路径规划效率对比（成功样本）', fontsize=14, fontweight='bold')
        ax.set_xticks(x)
        ax.set_xticklabels(grouped.index, rotation=45, ha='right', fontsize=8)
        ax.grid(axis='y', alpha=0.3)
        
        plt.tight_layout()
        output_path = self.output_dir / 'path_efficiency_comparison.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"已保存: {output_path}")
    
    def plot_success_vs_quality_tradeoff(self):
        """绘制成功率vs路径质量权衡图"""
        print("\n生成成功率-路径质量权衡图...")
        
        # 计算成功率
        success_rate = self.df.groupby('pid_config')['success'].mean()
        
        # 计算成功样本的平均路径长度
        df_success = self.df[self.df['success'] == True].copy()
        
        if len(df_success) == 0:
            print("警告: 没有成功的案例，跳过权衡图")
            return
        
        path_quality = df_success.groupby('pid_config')['path_length'].mean()
        
        # 合并
        tradeoff_df = pd.DataFrame({
            'success_rate': success_rate,
            'path_length': path_quality
        }).dropna()
        
        # 获取Kp值用于着色
        kp_values = self.df.groupby('pid_config')['Kp'].first()
        tradeoff_df = tradeoff_df.join(kp_values)
        
        fig, ax = plt.subplots(figsize=(10, 7))
        
        scatter = ax.scatter(
            tradeoff_df['success_rate'] * 100,
            tradeoff_df['path_length'],
            c=tradeoff_df['Kp'],
            s=150,
            alpha=0.7,
            cmap='coolwarm',
            edgecolors='black',
            linewidth=1.5
        )
        
        # 标注每个点
        for idx, row in tradeoff_df.iterrows():
            ax.annotate(
                idx,
                (row['success_rate'] * 100, row['path_length']),
                xytext=(5, 5),
                textcoords='offset points',
                fontsize=7,
                alpha=0.7
            )
        
        ax.set_xlabel('成功率 (%)', fontsize=12)
        ax.set_ylabel('平均路径长度（成功样本）', fontsize=12)
        ax.set_title('成功率 vs 路径质量权衡', fontsize=14, fontweight='bold')
        ax.grid(alpha=0.3)
        
        # 添加colorbar
        cbar = plt.colorbar(scatter, ax=ax)
        cbar.set_label('Kp 值', fontsize=11)
        
        # 标注理想区域（高成功率+短路径）
        ax.axvline(x=70, color='green', linestyle='--', alpha=0.5, label='成功率阈值70%')
        ax.legend()
        
        plt.tight_layout()
        output_path = self.output_dir / 'success_vs_quality_tradeoff.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"已保存: {output_path}")
    
    def plot_behavior_analysis(self):
        """绘制PID对搜索行为的影响分析"""
        print("\n生成搜索行为分析图...")
        
        df_success = self.df[self.df['success'] == True].copy()
        
        if len(df_success) == 0:
            print("警告: 没有成功的案例，跳过行为分析图")
            return
        
        df_success['path_efficiency'] = df_success['path_length'] / df_success['tree_nodes']
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        # 1. 树节点数对比
        ax = axes[0, 0]
        grouped = df_success.groupby('pid_config')['tree_nodes'].mean().sort_values()
        ax.barh(range(len(grouped)), grouped.values, alpha=0.7, color='steelblue')
        ax.set_yticks(range(len(grouped)))
        ax.set_yticklabels(grouped.index, fontsize=8)
        ax.set_xlabel('平均树节点数', fontsize=10)
        ax.set_title('搜索空间大小对比', fontsize=11, fontweight='bold')
        ax.grid(axis='x', alpha=0.3)
        
        # 2. 路径效率对比
        ax = axes[0, 1]
        grouped = df_success.groupby('pid_config')['path_efficiency'].mean().sort_values(ascending=False)
        colors = plt.cm.RdYlGn(np.linspace(0.3, 0.9, len(grouped)))
        ax.barh(range(len(grouped)), grouped.values, alpha=0.7, color=colors)
        ax.set_yticks(range(len(grouped)))
        ax.set_yticklabels(grouped.index, fontsize=8)
        ax.set_xlabel('单位迭代路径增长率', fontsize=10)
        ax.set_title('路径规划效率对比', fontsize=11, fontweight='bold')
        ax.grid(axis='x', alpha=0.3)
        
        # 3. 规划时间对比
        ax = axes[1, 0]
        grouped = df_success.groupby('pid_config')['planning_time'].mean().sort_values()
        ax.barh(range(len(grouped)), grouped.values, alpha=0.7, color='coral')
        ax.set_yticks(range(len(grouped)))
        ax.set_yticklabels(grouped.index, fontsize=8)
        ax.set_xlabel('平均规划时间 (s)', fontsize=10)
        ax.set_title('计算效率对比', fontsize=11, fontweight='bold')
        ax.grid(axis='x', alpha=0.3)
        
        # 4. 平滑度对比
        ax = axes[1, 1]
        grouped = df_success.groupby('pid_config')['smoothness'].mean().sort_values()
        ax.barh(range(len(grouped)), grouped.values, alpha=0.7, color='mediumseagreen')
        ax.set_yticks(range(len(grouped)))
        ax.set_yticklabels(grouped.index, fontsize=8)
        ax.set_xlabel('平均角度变化 (度)', fontsize=10)
        ax.set_title('路径平滑度对比', fontsize=11, fontweight='bold')
        ax.grid(axis='x', alpha=0.3)
        
        plt.tight_layout()
        output_path = self.output_dir / 'behavior_analysis.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"已保存: {output_path}")
    
    def plot_esr_comparison(self):
        """【新增】绘制有效采样比例(ESR)对比图 - 核心指标1"""
        if 'effective_sampling_ratio' not in self.df.columns:
            print("跳过ESR对比图: 数据中不包含effective_sampling_ratio字段")
            return
        
        print("\n生成有效采样比例(ESR)对比图...")
        
        fig, axes = plt.subplots(1, 2, figsize=(14, 5))
        
        # 1. ESR均值对比（柱状图）
        ax = axes[0]
        esr_mean = self.df.groupby('pid_config')['effective_sampling_ratio'].mean().sort_values(ascending=False)
        colors = ['#2ecc71' if 'PID' in name else '#e74c3c' for name in esr_mean.index]
        
        bars = ax.bar(range(len(esr_mean)), esr_mean.values * 100, color=colors, alpha=0.7, edgecolor='black')
        ax.set_xticks(range(len(esr_mean)))
        ax.set_xticklabels(esr_mean.index, rotation=15, ha='right', fontsize=9)
        ax.set_ylabel('有效采样比例 (%)', fontsize=11, fontweight='bold')
        ax.set_title('【核心指标1】有效采样比例(ESR)对比\n证明PID引导采样进入约束区域', 
                    fontsize=12, fontweight='bold')
        ax.grid(axis='y', alpha=0.3, linestyle='--')
        
        # 在柱子上标注数值
        for i, (bar, val) in enumerate(zip(bars, esr_mean.values)):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height,
                   f'{val*100:.1f}%',
                   ha='center', va='bottom', fontsize=9, fontweight='bold')
        
        # 2. 箱线图展示分布
        ax = axes[1]
        data_to_plot = []
        labels = []
        for config in esr_mean.index:
            data = self.df[self.df['pid_config'] == config]['effective_sampling_ratio'].values * 100
            data_to_plot.append(data)
            labels.append(config)
        
        bp = ax.boxplot(data_to_plot, labels=labels, patch_artist=True, showmeans=True)
        
        # 美化箱线图
        for patch, color in zip(bp['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.6)
        
        ax.set_ylabel('有效采样比例 (%)', fontsize=11, fontweight='bold')
        ax.set_title('ESR分布稳定性', fontsize=12, fontweight='bold')
        ax.grid(axis='y', alpha=0.3, linestyle='--')
        plt.setp(ax.get_xticklabels(), rotation=15, ha='right', fontsize=9)
        
        plt.tight_layout()
        output_path = self.output_dir / 'esr_comparison.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"已保存: {output_path}")
    
    def plot_sample_to_success(self):
        """【新增】绘制Sample-to-Success对比图 - 核心指标2"""
        if 'first_solution_iter' not in self.df.columns:
            print("跳过Sample-to-Success图: 数据中不包含first_solution_iter字段")
            return
        
        df_success = self.df[self.df['success'] == True].copy()
        if len(df_success) == 0:
            print("跳过Sample-to-Success图: 没有成功样本")
            return
        
        print("\n生成Sample-to-Success对比图...")
        
        fig, axes = plt.subplots(1, 2, figsize=(14, 5))
        
        # 1. 平均采样量对比（越小越好）
        ax = axes[0]
        sts_mean = df_success.groupby('pid_config')['first_solution_iter'].mean().sort_values()
        colors = ['#3498db' if 'Optimal' in name else '#95a5a6' for name in sts_mean.index]
        
        bars = ax.barh(range(len(sts_mean)), sts_mean.values, color=colors, alpha=0.7, edgecolor='black')
        ax.set_yticks(range(len(sts_mean)))
        ax.set_yticklabels(sts_mean.index, fontsize=9)
        ax.set_xlabel('平均采样次数（达到首次解）', fontsize=11, fontweight='bold')
        ax.set_title('【核心指标2】Sample-to-Success对比\n证明PID提高单位采样价值（越少越好）', 
                    fontsize=12, fontweight='bold')
        ax.grid(axis='x', alpha=0.3, linestyle='--')
        
        # 在条形图上标注数值
        for i, (bar, val) in enumerate(zip(bars, sts_mean.values)):
            width = bar.get_width()
            ax.text(width, bar.get_y() + bar.get_height()/2.,
                   f'{val:.0f}',
                   ha='left', va='center', fontsize=9, fontweight='bold', 
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.7))
        
        # 2. 采样效率提升比例
        ax = axes[1]
        if 'No_PID' in sts_mean.index:
            baseline = sts_mean['No_PID']
            improvement = {}
            for config in sts_mean.index:
                if config != 'No_PID':
                    reduction_pct = (baseline - sts_mean[config]) / baseline * 100
                    improvement[config] = reduction_pct
            
            if improvement:
                configs = list(improvement.keys())
                values = list(improvement.values())
                colors_imp = ['#27ae60' if v > 0 else '#e74c3c' for v in values]
                
                bars = ax.bar(range(len(configs)), values, color=colors_imp, alpha=0.7, edgecolor='black')
                ax.set_xticks(range(len(configs)))
                ax.set_xticklabels(configs, rotation=15, ha='right', fontsize=9)
                ax.set_ylabel('采样量减少 (%)', fontsize=11, fontweight='bold')
                ax.set_title('相对No_PID的效率提升', fontsize=12, fontweight='bold')
                ax.axhline(y=0, color='black', linestyle='--', linewidth=1)
                ax.grid(axis='y', alpha=0.3, linestyle='--')
                
                # 标注数值
                for bar, val in zip(bars, values):
                    height = bar.get_height()
                    ax.text(bar.get_x() + bar.get_width()/2., height,
                           f'{val:.1f}%',
                           ha='center', va='bottom' if val > 0 else 'top',
                           fontsize=9, fontweight='bold')
        
        plt.tight_layout()
        output_path = self.output_dir / 'sample_to_success.png'
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"已保存: {output_path}")
    
    def generate_all_plots(self):
        """生成所有图表"""
        print("\n" + "="*70)
        print("开始生成可视化图表")
        print("="*70)
        
        # ★★★ 优先生成核心指标图表 ★★★
        self.plot_esr_comparison()  # 新增：核心指标1
        self.plot_sample_to_success()  # 新增：核心指标2
        
        # 传统指标图表
        self.plot_success_rate_comparison()
        self.plot_path_length_comparison()
        self.plot_path_efficiency_comparison()
        self.plot_success_vs_quality_tradeoff()
        self.plot_behavior_analysis()
        self.plot_parameter_sensitivity()
        self.plot_boxplot_by_scenario()
        self.plot_pareto_frontier()
        
        print("\n" + "="*70)
        print("所有图表生成完成!")
        print("="*70)


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='分析PID实验结果')
    parser.add_argument('csv_file', type=str, help='CSV结果文件路径')
    
    args = parser.parse_args()
    
    # 创建分析器
    analyzer = ResultAnalyzer(args.csv_file)
    
    # 生成统计摘要
    analyzer.generate_summary_statistics()
    
    # 生成所有图表
    analyzer.generate_all_plots()


if __name__ == '__main__':
    main()
