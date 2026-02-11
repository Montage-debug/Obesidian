# -*- coding: utf-8 -*-
"""
简化实验运行脚本 - 修复编码问题
测试PID参数对路径规划效果的影响
"""

import sys
import os
from pathlib import Path

# 设置UTF-8编码
if sys.platform == 'win32':
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8')

# 添加src目录到Python路径
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))
sys.path.insert(0, str(current_dir / 'src'))

import numpy as np
import pandas as pd
import time
from datetime import datetime

from src.environment import EnvironmentConfig
from src.sc_rrt_basic_pid import SCRRTBasicPID


def print_header(text):
    """打印格式化的标题"""
    print("\n" + "=" * 70)
    print(text.center(70))
    print("=" * 70)


def run_single_test(env, pid_config, max_iterations=500, step_size=50.0, 
                    goal_threshold=50.0):
    """运行单次测试"""
    planner = SCRRTBasicPID(
        env=env,
        max_iterations=max_iterations,
        mode='custom_pid',
        Kp=pid_config['Kp'],
        Ki=pid_config['Ki'],
        Kd=pid_config['Kd'],
        step_size=step_size,
        goal_threshold=goal_threshold,
        verbose=False
    )
    
    start_time = time.time()
    path, tree, success, metrics = planner.plan()
    planning_time = time.time() - start_time
    
    result = {
        'success': success,
        'planning_time': planning_time,
        'iterations': metrics.get('iterations', 0),
        'tree_nodes': metrics.get('tree_nodes', 0),
    }
    
    if success:
        result.update({
            'path_length': metrics.get('path_length', 0),
            'smoothness': metrics.get('smoothness', 0),
            'convergence_iteration': metrics.get('convergence_iteration', 0),
            'effective_sampling_ratio': metrics.get('effective_sampling_ratio', 0),
            'sample_to_success': metrics.get('sample_to_success', 0),
        })
    else:
        result.update({
            'path_length': np.inf,
            'smoothness': np.inf,
            'convergence_iteration': max_iterations,
            'effective_sampling_ratio': metrics.get('effective_sampling_ratio', 0),
            'sample_to_success': max_iterations,
        })
    
    return result


def run_simple_experiment():
    """运行简化的PID对比实验"""
    print_header("SC-RRT PID参数对比实验")
    
    # 定义PID参数配置（拉开差距）
    pid_configs = [
        {'name': 'No_PID', 'Kp': 0.0, 'Ki': 0.0, 'Kd': 0.0},           # 无PID控制
        {'name': 'Low_PID', 'Kp': 0.1, 'Ki': 0.015, 'Kd': 0.04},       # 低增益
        {'name': 'Medium_PID', 'Kp': 0.25, 'Ki': 0.04, 'Kd': 0.10},    # 中等增益（预期最优）
        {'name': 'High_PID', 'Kp': 0.5, 'Ki': 0.075, 'Kd': 0.20},      # 高增益
    ]
    
    # 创建测试场景（2D高密度）
    print("\n创建测试场景...")
    env_2d = EnvironmentConfig.generate_2d_environment(
        bounds=[0, 1500, 0, 1500],
        num_obstacles=150,
        start_point=np.array([75, 75]),
        goal_point=np.array([1425, 1425]),
        radius_range=(20, 40),
        min_spacing=16,
        clearance=20,
        seed=301
    )
    
    # 实验参数
    num_trials = 10  # 每个配置运行10次
    max_iterations = 650
    
    print(f"\n实验配置:")
    print(f"  PID配置数: {len(pid_configs)}")
    print(f"  重复次数: {num_trials}")
    print(f"  最大迭代: {max_iterations}")
    print(f"  总实验数: {len(pid_configs) * num_trials}")
    
    # 运行实验
    results = []
    total_runs = len(pid_configs) * num_trials
    run_count = 0
    
    for pid_config in pid_configs:
        print(f"\n--- 测试配置: {pid_config['name']} ---")
        print(f"    Kp={pid_config['Kp']:.3f}, Ki={pid_config['Ki']:.3f}, Kd={pid_config['Kd']:.3f}")
        
        success_count = 0
        
        for trial in range(num_trials):
            run_count += 1
            print(f"  [{run_count}/{total_runs}] 运行 {trial + 1}/{num_trials}...", end=' ')
            
            result = run_single_test(
                env_2d,
                pid_config,
                max_iterations=max_iterations
            )
            
            result.update({
                'config_name': pid_config['name'],
                'Kp': pid_config['Kp'],
                'Ki': pid_config['Ki'],
                'Kd': pid_config['Kd'],
                'trial': trial + 1
            })
            
            results.append(result)
            
            if result['success']:
                success_count += 1
                print(f"成功 (路径长度: {result['path_length']:.1f}, ESR: {result['effective_sampling_ratio']:.3f})")
            else:
                print("失败")
        
        success_rate = success_count / num_trials * 100
        print(f"\n  配置 {pid_config['name']} 成功率: {success_rate:.1f}% ({success_count}/{num_trials})")
    
    # 保存结果
    df = pd.DataFrame(results)
    output_dir = Path('results')
    output_dir.mkdir(exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = output_dir / f'simple_experiment_{timestamp}.csv'
    df.to_csv(csv_path, index=False, encoding='utf-8-sig')
    
    print(f"\n\n结果已保存到: {csv_path}")
    
    # 打印汇总统计
    print_header("实验结果汇总")
    
    summary = df.groupby('config_name').agg({
        'success': ['mean', 'sum'],
        'path_length': ['mean', 'std'],
        'planning_time': ['mean', 'std'],
        'effective_sampling_ratio': ['mean', 'std'],
        'sample_to_success': ['mean', 'std'],
        'tree_nodes': ['mean', 'std']
    }).round(4)
    
    print("\n成功率统计:")
    print(summary[('success', 'mean')].apply(lambda x: f"{x*100:.1f}%"))
    
    print("\n路径长度统计 (成功样本):")
    success_df = df[df['success'] == True]
    if len(success_df) > 0:
        for config in pid_configs:
            config_data = success_df[success_df['config_name'] == config['name']]
            if len(config_data) > 0:
                mean_len = config_data['path_length'].mean()
                std_len = config_data['path_length'].std()
                print(f"  {config['name']:12s}: {mean_len:7.1f} ± {std_len:5.1f}")
    
    print("\n有效采样比例(ESR)统计:")
    for config in pid_configs:
        config_data = df[df['config_name'] == config['name']]
        mean_esr = config_data['effective_sampling_ratio'].mean()
        std_esr = config_data['effective_sampling_ratio'].std()
        print(f"  {config['name']:12s}: {mean_esr*100:5.1f}% ± {std_esr*100:4.1f}%")
    
    print("\n首次成功所需采样数统计 (成功样本):")
    if len(success_df) > 0:
        for config in pid_configs:
            config_data = success_df[success_df['config_name'] == config['name']]
            if len(config_data) > 0:
                mean_sts = config_data['sample_to_success'].mean()
                std_sts = config_data['sample_to_success'].std()
                print(f"  {config['name']:12s}: {mean_sts:6.0f} ± {std_sts:5.0f}")
    
    return df


if __name__ == '__main__':
    try:
        df = run_simple_experiment()
        print("\n实验完成!")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
