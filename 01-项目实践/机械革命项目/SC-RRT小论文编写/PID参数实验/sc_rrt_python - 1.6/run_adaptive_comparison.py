#!/usr/bin/env python3
"""
自适应采样控制器对比实验
对比有/无自适应控制器的性能差异

实验设计:
1. No_Adaptive: 固定椭球参数 (gamma=4.0, p=0.3)
2. Adaptive: 三阶段自适应控制 (Exploration→Exploitation→Convergence)

评估指标:
- 成功率 Success Rate
- 样本到成功数 Sample-to-Success  
- 路径平均长度 Path Length Mean
- 路径稳定性 CV (Coefficient of Variation)
- 采样效率 ESR (Ellipsoid Sampling Ratio)
"""

import sys
import numpy as np
import pandas as pd
from pathlib import Path
from datetime import datetime
import json

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent))

from src.environment import EnvironmentConfig
from src.adaptive_sampling_controller import AdaptiveSamplingController


def create_experiment_config():
    """创建实验配置"""
    return {
        "experiment_name": "adaptive_comparison",
        "timestamp": datetime.now().strftime("%Y%m%d_%H%M%S"),
        
        # 环境配置 (2D高密度)
        "env_config": {
            "bounds": [0, 1500, 0, 1500],
            "num_obstacles": 225,  # 高密度场景
            "radius_range": (22, 38),
            "start_point": np.array([75.0, 75.0]),
            "goal_point": np.array([1425.0, 1425.0]),
            "auto_adjust": True,
            "max_adjust_rounds": 6,
            "seed": None
        },
        
        # 实验组配置
        "test_groups": [
            {
                "name": "No_Adaptive",
                "mode": "no_adaptive",
                "description": "固定椭球参数基准",
                "adaptive_config": None,
                "color": "#1f77b4"  # 蓝色
            },
            {
                "name": "Adaptive",
                "mode": "adaptive",
                "description": "三阶段自适应控制",
                "adaptive_config": {
                    "gamma_explore": 6.0,
                    "p_explore": 0.2,
                    "gamma_exploit": 3.5,
                    "p_exploit": 0.5,
                    "gamma_converge": 2.0,
                    "p_converge": 0.7
                },
                "color": "#ff7f0e"  # 橙色
            }
        ],
        
        # 算法配置
        "algorithm_config": {
            "max_iterations": 500,
            "step_size": None,  # 自动设置
            "goal_threshold": None,  # 自动设置
            "update_interval": 50,
            "smoothing_factor": 0.7,
            "use_pareto": True,
            "pareto_interval": 100,
            "pareto_prob": 0.8
        },
        
        # 实验参数
        "experiment_params": {
            "repetitions": 30,  # 每组重复30次
            "random_seed_start": 42,
            "verbose": False  # 单次实验不打印详细信息
        }
    }


def run_single_experiment(env, start, goal, config, group_config, seed):
    """运行单次实验"""
    from src.sc_rrt_adaptive import SCRRTAdaptive
    
    np.random.seed(seed)
    
    # 创建算法实例
    planner = SCRRTAdaptive(
        env=env,
        max_iterations=config['algorithm_config']['max_iterations'],
        mode=group_config['mode'],
        adaptive_config=group_config['adaptive_config'],
        step_size=config['algorithm_config']['step_size'],
        goal_threshold=config['algorithm_config']['goal_threshold'],
        update_interval=config['algorithm_config']['update_interval'],
        smoothing_factor=config['algorithm_config']['smoothing_factor'],
        use_pareto=config['algorithm_config']['use_pareto'],
        pareto_interval=config['algorithm_config']['pareto_interval'],
        pareto_prob=config['algorithm_config']['pareto_prob'],
        verbose=config['experiment_params']['verbose']
    )
    
    # 规划路径
    result = planner.plan(start, goal)
    
    # 提取关键指标
    metrics = {
        'success': result['success'],
        'path_length': result.get('path_length', np.nan),
        'samples_to_success': result.get('samples_to_success', np.nan),
        'total_samples': result.get('total_samples', 0),
        'time_elapsed': result.get('time_elapsed', np.nan),
        'tree_size': result.get('tree_size', 0),
        'esr': result.get('ESR', np.nan)
    }
    
    return metrics


def run_experiment_group(env, config, group_config):
    """运行实验组"""
    print(f"\n{'='*60}")
    print(f"运行实验组: {group_config['name']}")
    print(f"描述: {group_config['description']}")
    print(f"模式: {group_config['mode']}")
    print(f"{'='*60}")
    
    results = []
    repetitions = config['experiment_params']['repetitions']
    seed_start = config['experiment_params']['random_seed_start']
    
    start = env['start']
    goal = env['goal']
    
    for i in range(repetitions):
        seed = seed_start + i
        print(f"  [{i+1}/{repetitions}] 种子={seed}...", end='', flush=True)
        
        try:
            metrics = run_single_experiment(env, start, goal, config, group_config, seed)
            metrics['seed'] = seed
            metrics['run_id'] = i + 1
            results.append(metrics)
            
            status = "✓成功" if metrics['success'] else "✗失败"
            if metrics['success']:
                print(f" {status} | 路径={metrics['path_length']:.1f} | 采样={metrics['samples_to_success']:.0f}")
            else:
                print(f" {status}")
                
        except Exception as e:
            print(f" ✗错误: {str(e)}")
            results.append({
                'seed': seed,
                'run_id': i + 1,
                'success': False,
                'path_length': np.nan,
                'samples_to_success': np.nan,
                'total_samples': config['algorithm_config']['max_iterations'],
                'time_elapsed': np.nan,
                'tree_size': 0,
                'esr': np.nan,
                'error': str(e)
            })
    
    df = pd.DataFrame(results)
    return df


def analyze_results(results_dict):
    """分析实验结果"""
    print(f"\n{'='*80}")
    print("实验结果分析")
    print(f"{'='*80}")
    
    summary = []
    
    for group_name, df in results_dict.items():
        successful = df[df['success'] == True]
        
        if len(successful) > 0:
            stats = {
                'Group': group_name,
                'Success_Rate (%)': len(successful) / len(df) * 100,
                'Path_Length_Mean': successful['path_length'].mean(),
                'Path_Length_Std': successful['path_length'].std(),
                'Path_Length_CV (%)': (successful['path_length'].std() / successful['path_length'].mean()) * 100,
                'Samples_Mean': successful['samples_to_success'].mean(),
                'Samples_Std': successful['samples_to_success'].std(),
                'Time_Mean (s)': successful['time_elapsed'].mean(),
                'ESR_Mean (%)': successful['esr'].mean() * 100 if 'esr' in successful.columns else np.nan,
                'Runs': len(df),
                'Success_Count': len(successful)
            }
        else:
            stats = {
                'Group': group_name,
                'Success_Rate (%)': 0.0,
                'Path_Length_Mean': np.nan,
                'Path_Length_Std': np.nan,
                'Path_Length_CV (%)': np.nan,
                'Samples_Mean': np.nan,
                'Samples_Std': np.nan,
                'Time_Mean (s)': np.nan,
                'ESR_Mean (%)': np.nan,
                'Runs': len(df),
                'Success_Count': 0
            }
        
        summary.append(stats)
    
    summary_df = pd.DataFrame(summary)
    
    # 打印摘要表格
    print("\n" + "="*80)
    print("关键指标对比")
    print("="*80)
    
    for _, row in summary_df.iterrows():
        print(f"\n【{row['Group']}】")
        print(f"  成功率:        {row['Success_Rate (%)']:.1f}% ({row['Success_Count']}/{row['Runs']})")
        print(f"  路径长度:      {row['Path_Length_Mean']:.1f} ± {row['Path_Length_Std']:.1f}")
        print(f"  路径CV:        {row['Path_Length_CV (%)']:.2f}%")
        print(f"  样本到成功:    {row['Samples_Mean']:.0f} ± {row['Samples_Std']:.0f}")
        print(f"  平均时间:      {row['Time_Mean (s)']:.2f}s")
        if not np.isnan(row['ESR_Mean (%)']):
            print(f"  采样效率(ESR): {row['ESR_Mean (%)']:.1f}%")
    
    # 计算性能提升
    if len(summary_df) == 2:
        print(f"\n{'='*80}")
        print("性能提升分析 (Adaptive vs No_Adaptive)")
        print(f"{'='*80}")
        
        baseline = summary_df[summary_df['Group'] == 'No_Adaptive'].iloc[0]
        adaptive = summary_df[summary_df['Group'] == 'Adaptive'].iloc[0]
        
        print(f"  成功率提升:    {adaptive['Success_Rate (%)'] - baseline['Success_Rate (%)']:+.1f}%")
        
        if not np.isnan(adaptive['Path_Length_Mean']) and not np.isnan(baseline['Path_Length_Mean']):
            path_improve = (baseline['Path_Length_Mean'] - adaptive['Path_Length_Mean']) / baseline['Path_Length_Mean'] * 100
            print(f"  路径长度改善:  {path_improve:+.2f}%")
            
            cv_improve = baseline['Path_Length_CV (%)'] - adaptive['Path_Length_CV (%)']
            print(f"  稳定性提升:    CV {cv_improve:+.2f}%")
            
            samples_improve = (baseline['Samples_Mean'] - adaptive['Samples_Mean']) / baseline['Samples_Mean'] * 100
            print(f"  采样效率提升:  {samples_improve:+.2f}%")
    
    return summary_df


def save_results(config, results_dict, summary_df):
    """保存实验结果"""
    results_dir = Path("results") / config['experiment_name']
    results_dir.mkdir(parents=True, exist_ok=True)
    
    timestamp = config['timestamp']
    
    # 保存配置
    config_path = results_dir / f"config_{timestamp}.json"
    with open(config_path, 'w', encoding='utf-8') as f:
        # 转换numpy数组为列表以便JSON序列化
        config_serializable = {
            k: v.tolist() if isinstance(v, np.ndarray) else v
            for k, v in config.items()
        }
        json.dump(config_serializable, f, indent=2, ensure_ascii=False)
    
    # 保存详细结果
    for group_name, df in results_dict.items():
        csv_path = results_dir / f"{group_name}_{timestamp}.csv"
        df.to_csv(csv_path, index=False)
        print(f"\n保存详细结果: {csv_path}")
    
    # 保存摘要
    summary_path = results_dir / f"summary_{timestamp}.csv"
    summary_df.to_csv(summary_path, index=False)
    print(f"保存摘要结果: {summary_path}")
    
    print(f"\n所有结果已保存到: {results_dir}")


def main():
    """主函数"""
    print("="*80)
    print(" SC-RRT 自适应采样控制器对比实验")
    print("="*80)
    
    # 创建实验配置
    config = create_experiment_config()
    
    print(f"\n实验配置:")
    print(f"  环境维度:      2D")
    print(f"  障碍物数量:    {config['env_config']['num_obstacles']}")
    print(f"  最大迭代:      {config['algorithm_config']['max_iterations']}")
    print(f"  重复次数:      {config['experiment_params']['repetitions']}")
    print(f"  实验组数:      {len(config['test_groups'])}")
    
    # 创建环境
    print(f"\n创建实验环境...")
    env = EnvironmentConfig.generate_2d_environment(**config['env_config'])
    print(f"  ✓ 环境创建完成: {len(env['obstacles'])} 个障碍物")
    
    print(f"  起点: {env['start']}")
    print(f"  终点: {env['goal']}")
    
    # 运行所有实验组
    results_dict = {}
    for group_config in config['test_groups']:
        df = run_experiment_group(env, config, group_config)
        results_dict[group_config['name']] = df
    
    # 分析结果
    summary_df = analyze_results(results_dict)
    
    # 保存结果
    save_results(config, results_dict, summary_df)
    
    print(f"\n{'='*80}")
    print("实验完成!")
    print(f"{'='*80}\n")


if __name__ == "__main__":
    main()
