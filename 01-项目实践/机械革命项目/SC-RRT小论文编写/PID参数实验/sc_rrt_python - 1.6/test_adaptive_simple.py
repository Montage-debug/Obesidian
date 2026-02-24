#!/usr/bin/env python3
"""
简化版自适应控制器对比实验
直接运行SC-RRT算法,对比有无自适应控制的效果
"""

import sys
import numpy as np
import pandas as pd
from pathlib import Path
from datetime import datetime
import time

sys.path.insert(0, str(Path(__file__).parent))

from src.environment import EnvironmentConfig
from src.sc_rrt_basic_pid import SCRRTBasicPID


def main():
    print("="*80)
    print(" SC-RRT 自适应采样控制器简化对比实验")
    print("="*80)
    
    # 创建高密度2D环境
    print("\n创建测试环境...")
    env = EnvironmentConfig.generate_2d_environment(
        bounds=[0, 1500, 0, 1500],
        num_obstacles=225,
        start_point=np.array([75.0, 75.0]),
        goal_point=np.array([1425.0, 1425.0]),
        radius_range=(22, 38),
        auto_adjust=True,
        max_adjust_rounds=6,
        seed=42
    )
    
    print(f"  ✓ 环境创建完成: {len(env['obstacles'])} 个障碍物")
    print(f"  起点: {env['start']}")
    print(f"  终点: {env['goal']}")
    
    # 实验参数
    max_iterations = 500
    repetitions = 5  # 先测试5次，确保能正常运行
    
    results = []
    
    # 测试1: No_Adaptive模式
    print(f"\n{'='*80}")
    print("测试1: No_Adaptive (固定椭球参数)")
    print(f"{'='*80}")
    
    for i in range(repetitions):
        seed = 42 + i
        np.random.seed(seed)
        print(f"\n  运行 [{i+1}/{repetitions}] seed={seed}")
        
        planner = SCRRTBasicPID(
            env=env,
            max_iterations=max_iterations,
            mode='no_adaptive',
            verbose=True
        )
        
        start_time = time.time()
        path, tree, success, metrics = planner.plan(env['start'], env['goal'])
        elapsed = time.time() - start_time
        
        result = {
            'mode': 'no_adaptive',
            'run': i+1,
            'seed': seed,
            'success': success,
            'time': elapsed
        }
        
        if success:
            result.update({
                'path_length': metrics.get('path_length', np.nan),
                'samples_to_success': metrics.get('samples_to_success', np.nan),
                'total_samples': metrics.get('total_samples', 0)
            })
            print(f"    ✓ 成功 | 路径长度={result['path_length']:.1f} | "
                  f"采样={result['samples_to_success']:.0f} | 时间={elapsed:.2f}s")
        else:
            result.update({
                'path_length': np.nan,
                'samples_to_success': np.nan,
                'total_samples': max_iterations
            })
            print(f"    ✗ 失败 | 时间={elapsed:.2f}s")
        
        results.append(result)
    
    # 测试2: Adaptive模式
    print(f"\n{'='*80}")
    print("测试2: Adaptive (三阶段自适应控制)")
    print(f"{'='*80}")
    
    adaptive_config = {
        "gamma_explore": 6.0,
        "p_explore": 0.2,
        "gamma_exploit": 3.5,
        "p_exploit": 0.5,
        "gamma_converge": 2.0,
        "p_converge": 0.7
    }
    
    for i in range(repetitions):
        seed = 42 + i
        np.random.seed(seed)
        print(f"\n  运行 [{i+1}/{repetitions}] seed={seed}")
        
        planner = SCRRTBasicPID(
            env=env,
            max_iterations=max_iterations,
            mode='adaptive',
            adaptive_config=adaptive_config,
            verbose=True
        )
        
        start_time = time.time()
        path, tree, success, metrics = planner.plan(env['start'], env['goal'])
        elapsed = time.time() - start_time
        
        result = {
            'mode': 'adaptive',
            'run': i+1,
            'seed': seed,
            'success': success,
            'time': elapsed
        }
        
        if success:
            result.update({
                'path_length': metrics.get('path_length', np.nan),
                'samples_to_success': metrics.get('samples_to_success', np.nan),
                'total_samples': metrics.get('total_samples', 0)
            })
            print(f"    ✓ 成功 | 路径长度={result['path_length']:.1f} | "
                  f"采样={result['samples_to_success']:.0f} | 时间={elapsed:.2f}s")
        else:
            result.update({
                'path_length': np.nan,
                'samples_to_success': np.nan,
                'total_samples': max_iterations
            })
            print(f"    ✗ 失败 | 时间={elapsed:.2f}s")
        
        results.append(result)
    
    # 分析结果
    df = pd.DataFrame(results)
    
    print(f"\n{'='*80}")
    print("实验结果汇总")
    print(f"{'='*80}\n")
    
    for mode in ['no_adaptive', 'adaptive']:
        mode_df = df[df['mode'] == mode]
        successful = mode_df[mode_df['success'] == True]
        
        print(f"【{mode.upper()}】")
        print(f"  成功率: {len(successful)}/{len(mode_df)} = "
              f"{len(successful)/len(mode_df)*100:.1f}%")
        
        if len(successful) > 0:
            print(f"  路径长度: {successful['path_length'].mean():.1f} ± "
                  f"{successful['path_length'].std():.1f}")
            print(f"  样本数: {successful['samples_to_success'].mean():.0f} ± "
                  f"{successful['samples_to_success'].std():.0f}")
            print(f"  时间: {successful['time'].mean():.2f}s ± "
                  f"{successful['time'].std():.2f}s")
        print()
    
    # 保存结果
    results_dir = Path("results") / "simple_test"
    results_dir.mkdir(parents=True, exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = results_dir / f"results_{timestamp}.csv"
    df.to_csv(csv_path, index=False)
    
    print(f"结果已保存到: {csv_path}")
    print("\n实验完成!")


if __name__ == "__main__":
    main()
