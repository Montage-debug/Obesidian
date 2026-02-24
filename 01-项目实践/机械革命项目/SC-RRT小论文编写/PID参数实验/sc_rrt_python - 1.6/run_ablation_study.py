# -*- coding: utf-8 -*-
"""
SC-RRT 消融实验 (Ablation Study)
Ablation Study for SC-RRT Algorithm

目的：量化评估各创新组件的贡献
Purpose: Quantify the contribution of each innovative component

实验设计：
1. Informed-RRT* (基线) - 单向树 + 静态椭球约束
2. SC-RRT_Static - 双向树 + 静态椭球约束
3. SC-RRT_Adaptive - 双向树 + 自适应椭球约束 (完整算法)

评价指标：
- 成功率 (Success Rate)
- 收敛速度 (Iterations to Success)
- 路径质量 (Path Length, Smoothness)
- 计算效率 (Planning Time)
- 采样效率 (Effective Sampling Ratio)
"""

import sys
from pathlib import Path
import argparse
import warnings
import time
import numpy as np
import pandas as pd
from datetime import datetime
import json

warnings.filterwarnings('ignore')

# 添加路径
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))
sys.path.insert(0, str(current_dir / 'src'))

from src.environment import EnvironmentConfig
from src.sc_rrt_adaptive import SCRRTAdaptive


class AblationStudy:
    """消融实验类"""
    
    def _log(self, message):
        """同时输出到终端和日志文件"""
        print(message, end='', flush=True)  # 强制刷新输出
        if hasattr(self, 'log_handle') and self.log_handle:
            self.log_handle.write(message)
            self.log_handle.flush()  # 强制刷新文件
    
    def __init__(self, quick_test=False, output_dir='results/ablation'):
        self.quick_test = quick_test
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 实验配置
        if quick_test:
            self.num_trials = 30  # 快速测试（30次提高统计可靠性）
            self.max_iterations_2d = 1800  # 2D迭代提升（提高成功率至85-95%）
            self.max_iterations_3d = 800   # 3D迭代调整（增加难度，提升区分度）
        else:
            self.num_trials = 50  # 完整实验（50次满足SCI要求）
            self.max_iterations_2d = 2000  # 2D完整模式
            self.max_iterations_3d = 1000  # 3D完整模式
        
        # 创建测试场景
        self.scenarios = self._create_scenarios()
        
        # 创建算法配置
        self.algorithm_configs = self._create_algorithm_configs()
        
        # 日志
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.log_file = self.output_dir / f"ablation_log_{timestamp}.txt"
        self.results_file = self.output_dir / f"ablation_results_{timestamp}.csv"
        
        # 打开日志文件用于实时写入
        self.log_handle = open(self.log_file, 'w', encoding='utf-8', buffering=1)
        self._log(f"消融实验日志 - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        self._log("=" * 80 + "\n")
        self._log("快速消融实验模式\n" if quick_test else "完整消融实验模式\n")
        self._log("=" * 80 + "\n")
        self._log(f"重复次数: {self.num_trials}\n")
        self._log(f"二维迭代: {self.max_iterations_2d}\n")
        self._log(f"三维迭代: {self.max_iterations_3d}\n")
        self._log("=" * 80 + "\n\n")
    
    def _create_scenarios(self):
        """创建测试场景"""
        scenarios = []
        
        # 二维高密度场景（225个障碍物保持不变）
        scenarios.append({
            'name': '2D_HighDensity',
            'env': EnvironmentConfig.generate_2d_environment(
                num_obstacles=225,  # 高密度挑战场景
                bounds=[0, 1500, 0, 1500]
            ),
            'max_iterations': self.max_iterations_2d,
            'dimension': '2D'
        })
        
        # 三维高密度场景（400个障碍物保持不变）
        scenarios.append({
            'name': '3D_HighDensity',
            'env': EnvironmentConfig.generate_3d_environment(
                num_obstacles=400,  # 高密度挑战场景
                bounds=[0, 1500, 0, 1500, 0, 1500]
            ),
            'max_iterations': self.max_iterations_3d,
            'dimension': '3D'
        })
        
        return scenarios
    
    def _create_algorithm_configs(self):
        """创建算法配置"""
        configs = []
        
        # 配置1: Informed-RRT* (基线)
        # 单向树 + 静态椭球约束
        configs.append({
            'name': 'Informed-RRT*',
            'short_name': 'IRRT',
            'mode': 'informed_rrt_star',  # 特殊模式标记
            'description': '单向树 + 静态椭球约束 (基线算法)',
            'adaptive_config': None,
            'fixed_gamma': 1.0,  # Informed-RRT*使用紧约束
            'fixed_p': 1.0,      # 完全使用informed采样
            'color': '#E74C3C'   # 红色
        })
        
        # 配置2: SC-RRT Static (无自适应)
        # 双向树 + 静态椭球约束
        configs.append({
            'name': 'SC-RRT_Static',
            'short_name': 'SC-Static',
            'mode': 'no_adaptive',
            'description': '双向树 + 静态椭球约束',
            'adaptive_config': None,
            'fixed_gamma': 4.0,  # 中等约束强度
            'fixed_p': 0.3,
            'color': '#F39C12'   # 橙色
        })
        
        # 配置3: SC-RRT Adaptive (完整算法)
        # 双向树 + 自适应椭球约束
        configs.append({
            'name': 'SC-RRT_Adaptive',
            'short_name': 'SC-Adaptive',
            'mode': 'adaptive',
            'description': '双向树 + 自适应椭球约束 (完整算法)',
            'adaptive_config': {
                'gamma_explore': 6.0,
                'p_explore': 0.2,
                'gamma_exploit': 3.5,
                'p_exploit': 0.5,
                'gamma_converge': 2.0,
                'p_converge': 0.7
            },
            'color': '#27AE60'   # 绿色
        })
        
        return configs
    
    def run_single_trial(self, scenario, algo_config, trial_id):
        """运行单次实验"""
        try:
            # 创建规划器
            planner = SCRRTAdaptive(
                env=scenario['env'],
                max_iterations=scenario['max_iterations'],
                mode=algo_config['mode'],
                adaptive_config=algo_config.get('adaptive_config'),
                verbose=False
            )
            
            # 如果是Informed-RRT*模式，需要特殊处理
            if algo_config['mode'] == 'informed_rrt_star':
                # 设置为单向模式（通过修改内部参数）
                planner._use_bidirectional = False
            
            # 执行规划
            path, tree, success, metrics = planner.plan()
            
            # 整理结果
            result = {
                'scenario': scenario['name'],
                'dimension': scenario['dimension'],
                'algorithm': algo_config['name'],
                'short_name': algo_config['short_name'],
                'trial_id': trial_id,
                'success': success,
                'iterations': metrics['iterations'],
                'path_length': metrics['path_length'],
                'planning_time': metrics['planning_time'],
                'tree_nodes': metrics['tree_nodes'],
                'first_solution_iter': metrics.get('first_solution_iter', np.inf),
                'convergence_time': metrics.get('convergence_time', np.inf),
                'effective_sampling_ratio': metrics.get('effective_sampling_ratio', 0.0),
                'smoothness': metrics.get('smoothness', np.inf),
                'clearance': metrics.get('clearance', 0.0),
            }
            
            return result
            
        except Exception as e:
            error_msg = f"  错误: {str(e)}\n"
            self._log(error_msg)
            return {
                'scenario': scenario['name'],
                'dimension': scenario['dimension'],
                'algorithm': algo_config['name'],
                'short_name': algo_config['short_name'],
                'trial_id': trial_id,
                'success': False,
                'iterations': scenario['max_iterations'],
                'path_length': np.inf,
                'planning_time': 0,
                'tree_nodes': 0,
                'first_solution_iter': np.inf,
                'convergence_time': np.inf,
                'effective_sampling_ratio': 0.0,
                'smoothness': np.inf,
                'clearance': 0.0,
            }
    
    def run_experiment(self):
        """运行完整消融实验"""
        self._log("\n" + "=" * 80 + "\n")
        self._log("开始消融实验 (Ablation Study)\n")
        self._log(f"时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        self._log("=" * 80 + "\n")
        
        self._log(f"\n配置概览:\n")
        for i, config in enumerate(self.algorithm_configs, 1):
            self._log(f"  {i}. {config['name']}\n")
            self._log(f"     {config['description']}\n")
        
        self._log(f"\n场景概览:\n")
        for i, scenario in enumerate(self.scenarios, 1):
            self._log(f"  {i}. {scenario['name']} - {scenario['dimension']}\n")
        
        self._log(f"\n总实验数: {len(self.scenarios)} × {len(self.algorithm_configs)} × {self.num_trials} = {len(self.scenarios) * len(self.algorithm_configs) * self.num_trials}\n")
        self._log("=" * 80 + "\n\n")
        
        results = []
        total_runs = len(self.scenarios) * len(self.algorithm_configs) * self.num_trials
        current_run = 0
        start_time = time.time()
        
        for scenario in self.scenarios:
            self._log(f"\n{'=' * 80}\n")
            self._log(f"场景: {scenario['name']} ({scenario['dimension']})\n")
            self._log(f"迭代限制: {scenario['max_iterations']}\n")
            self._log(f"{'=' * 80}\n")
            
            for algo_config in self.algorithm_configs:
                self._log(f"\n算法: {algo_config['name']}\n")
                self._log(f"  {algo_config['description']}\n")
                
                success_count = 0
                
                for trial in range(self.num_trials):
                    current_run += 1
                    
                    result = self.run_single_trial(scenario, algo_config, trial + 1)
                    results.append(result)
                    
                    if result['success']:
                        success_count += 1
                    
                    # 计算进度
                    elapsed = time.time() - start_time
                    avg_time = elapsed / current_run
                    remaining = (total_runs - current_run) * avg_time
                    
                    status = "[OK]" if result['success'] else "[FAIL]"
                    self._log(f"  Trial {trial+1}/{self.num_trials}: {status} "
                          f"[{current_run}/{total_runs}, "
                          f"Elapsed {elapsed/60:.1f}min, Remaining {remaining/60:.1f}min]\n")
                
                self._log(f"  >> 成功率: {success_count}/{self.num_trials} ({100*success_count/self.num_trials:.1f}%)\n")
        
        # 保存结果
        df = pd.DataFrame(results)
        df.to_csv(self.results_file, index=False)
        
        self._log(f"\n{'=' * 80}\n")
        self._log("实验完成!\n")
        self._log(f"总耗时: {(time.time() - start_time)/60:.1f} 分钟\n")
        self._log(f"结果已保存: {self.results_file}\n")
        self._log(f"{'=' * 80}\n\n")
        
        # 生成统计分析
        self._generate_analysis(df)
        
        # 关闭日志文件
        if hasattr(self, 'log_handle') and self.log_handle:
            self.log_handle.close()
        
        return df
    
    def _generate_analysis(self, df):
        """生成统计分析报告"""
        self._log("\n" + "=" * 80 + "\n")
        self._log("消融实验统计分析报告\n")
        self._log("=" * 80 + "\n")
        
        # 按算法和场景分组统计
        for scenario_name in df['scenario'].unique():
            self._log(f"\n【{scenario_name}】\n")
            self._log("-" * 80 + "\n")
            
            scenario_df = df[df['scenario'] == scenario_name]
            
            # 统计表格
            stats = []
            for algo in self.algorithm_configs:
                algo_df = scenario_df[scenario_df['algorithm'] == algo['name']]
                success_df = algo_df[algo_df['success'] == True]
                
                if len(success_df) > 0:
                    stats.append({
                        '算法': algo['short_name'],
                        '成功率(%)': f"{100 * len(success_df) / len(algo_df):.1f}",
                        '平均迭代': f"{success_df['iterations'].mean():.0f}",
                        '平均路径长度': f"{success_df['path_length'].mean():.1f}",
                        '平均时间(s)': f"{success_df['planning_time'].mean():.2f}",
                        '采样效率(%)': f"{100 * success_df['effective_sampling_ratio'].mean():.1f}"
                    })
                else:
                    stats.append({
                        '算法': algo['short_name'],
                        '成功率(%)': '0.0',
                        '平均迭代': '-',
                        '平均路径长度': '-',
                        '平均时间(s)': '-',
                        '采样效率(%)': '-'
                    })
            
            stats_df = pd.DataFrame(stats)
            self._log(stats_df.to_string(index=False) + "\n")
        
        self._log("\n" + "=" * 80 + "\n")
        
        # 保存详细分析
        analysis_file = self.output_dir / f"analysis_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        with open(analysis_file, 'w', encoding='utf-8') as f:
            f.write("=" * 80 + "\n")
            f.write("SC-RRT 消融实验分析报告\n")
            f.write("=" * 80 + "\n\n")
            
            for scenario_name in df['scenario'].unique():
                f.write(f"\n【{scenario_name}】\n")
                f.write("-" * 80 + "\n")
                
                scenario_df = df[df['scenario'] == scenario_name]
                
                for algo in self.algorithm_configs:
                    algo_df = scenario_df[scenario_df['algorithm'] == algo['name']]
                    success_df = algo_df[algo_df['success'] == True]
                    
                    f.write(f"\n{algo['name']}:\n")
                    f.write(f"  成功率: {len(success_df)}/{len(algo_df)} ({100*len(success_df)/len(algo_df):.1f}%)\n")
                    
                    if len(success_df) > 0:
                        f.write(f"  迭代次数: {success_df['iterations'].mean():.1f} ± {success_df['iterations'].std():.1f}\n")
                        f.write(f"  路径长度: {success_df['path_length'].mean():.1f} ± {success_df['path_length'].std():.1f}\n")
                        f.write(f"  规划时间: {success_df['planning_time'].mean():.2f}s ± {success_df['planning_time'].std():.2f}s\n")
                        f.write(f"  采样效率: {100*success_df['effective_sampling_ratio'].mean():.1f}%\n")
        
        self._log(f"详细分析已保存: {analysis_file}\n")


def main():
    parser = argparse.ArgumentParser(description='SC-RRT 消融实验')
    parser.add_argument('--quick', action='store_true', help='快速测试模式')
    parser.add_argument('--output', type=str, default='results/ablation', help='输出目录')
    
    args = parser.parse_args()
    
    # 创建并运行实验
    study = AblationStudy(
        quick_test=args.quick,
        output_dir=args.output
    )
    
    study.run_experiment()


if __name__ == '__main__':
    main()
