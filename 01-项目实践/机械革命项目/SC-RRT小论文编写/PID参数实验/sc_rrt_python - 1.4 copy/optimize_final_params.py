# -*- coding: utf-8 -*-
"""
Fine-grained optimization: PID + Pareto Interval
Based on paper's suggestion: interval=4 is optimal
"""

import sys
from pathlib import Path
import numpy as np
import pandas as pd

current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir / 'src'))

from src.environment import EnvironmentConfig
from src.sc_rrt_basic_pid import SCRRTBasicPID


def fine_tune_optimization():
    """Fine-grained parameter optimization"""
    
    env = EnvironmentConfig.generate_2d_environment(
        bounds=[0, 800, 0, 800], num_obstacles=15,
        start_point=np.array([50, 50]), goal_point=np.array([750, 750]),
        radius_range=(10.0, 25.0), seed=42
    )
    
    # Test configurations with fine-grained intervals
    configs = [
        # Paper's optimal interval=4
        {'name': 'LowPID_Int4', 'Kp': 0.20, 'Ki': 0.03, 'Kd': 0.08, 'interval': 4},
        {'name': 'MidPID_Int4', 'Kp': 0.25, 'Ki': 0.04, 'Kd': 0.10, 'interval': 4},
        
        # Fine-tune around interval=4
        {'name': 'LowPID_Int3', 'Kp': 0.20, 'Ki': 0.03, 'Kd': 0.08, 'interval': 3},
        {'name': 'LowPID_Int5', 'Kp': 0.20, 'Ki': 0.03, 'Kd': 0.08, 'interval': 5},
        {'name': 'LowPID_Int6', 'Kp': 0.20, 'Ki': 0.03, 'Kd': 0.08, 'interval': 6},
        {'name': 'LowPID_Int8', 'Kp': 0.20, 'Ki': 0.03, 'Kd': 0.08, 'interval': 8},
        
        # Fine-tune PID with interval=4
        {'name': 'VeryLowPID_Int4', 'Kp': 0.18, 'Ki': 0.025, 'Kd': 0.07, 'interval': 4},
        {'name': 'FinePID1_Int4', 'Kp': 0.22, 'Ki': 0.035, 'Kd': 0.09, 'interval': 4},
        {'name': 'FinePID2_Int4', 'Kp': 0.20, 'Ki': 0.03, 'Kd': 0.10, 'interval': 4},
        {'name': 'FinePID3_Int4', 'Kp': 0.20, 'Ki': 0.04, 'Kd': 0.08, 'interval': 4},
        
        # Baseline for comparison
        {'name': 'Baseline_NoPareto', 'Kp': 0.25, 'Ki': 0.04, 'Kd': 0.10, 'interval': 0},
    ]
    
    print("\n" + "="*90)
    print("FINE-GRAINED OPTIMIZATION: PID + Pareto Interval")
    print("="*90)
    print("Paper reference: interval=4 achieves 0.034s with path~1157")
    print("Our best so far: LowPID(0.2,0.03,0.08) + interval=100 -> 1133.19")
    print("="*90)
    
    results = []
    
    for cfg in configs:
        print(f"\n[{cfg['name']}] PID({cfg['Kp']},{cfg['Ki']},{cfg['Kd']}) Int={cfg['interval']}")
        
        run_data = []
        for i in range(5):  # 5 runs per config
            planner = SCRRTBasicPID(
                env=env,
                max_iterations=5000,
                mode='custom_pid',
                Kp=cfg['Kp'], Ki=cfg['Ki'], Kd=cfg['Kd'],
                use_pareto=(cfg['interval'] > 0),
                pareto_interval=cfg['interval'] if cfg['interval'] > 0 else 100,
                pareto_prob=0.8,
                verbose=False
            )
            
            path, tree, success, metrics = planner.plan()
            
            run_data.append({
                'success': success,
                'length': metrics['path_length'] if success else np.inf,
                'time': metrics['planning_time'],
                'iter': metrics['iterations'],
                'pareto_count': len(planner.pareto_selections)
            })
            
            status = f"{metrics['path_length']:.0f}" if success else "FAIL"
            print(f"  Run{i+1}: {status}", end=' ', flush=True)
        
        # Statistics
        success_rate = sum(r['success'] for r in run_data) / len(run_data)
        
        if success_rate > 0:
            successful = [r for r in run_data if r['success']]
            avg_len = np.mean([r['length'] for r in successful])
            std_len = np.std([r['length'] for r in successful])
            avg_time = np.mean([r['time'] for r in successful])
            avg_pareto = np.mean([r['pareto_count'] for r in successful])
            
            results.append({
                'config': cfg['name'],
                'Kp': cfg['Kp'], 'Ki': cfg['Ki'], 'Kd': cfg['Kd'],
                'interval': cfg['interval'],
                'success_rate': success_rate,
                'avg_length': avg_len,
                'std_length': std_len,
                'avg_time': avg_time,
                'avg_pareto': avg_pareto
            })
            
            print(f"\n  => Avg: {avg_len:.1f}±{std_len:.1f}, Time: {avg_time:.3f}s, Pareto: {avg_pareto:.0f}")
        else:
            print("\n  => All failed")
    
    # Summary table
    print("\n" + "="*110)
    print("OPTIMIZATION RESULTS SUMMARY")
    print("="*110)
    
    df = pd.DataFrame(results)
    df = df.sort_values('avg_length')
    
    print(f"{'Configuration':<22} {'PID(Kp,Ki,Kd)':<20} {'Int':<5} {'Succ%':<7} "
          f"{'Path Length':<18} {'Time(s)':<10} {'Pareto#':<8}")
    print("-"*110)
    
    for _, row in df.iterrows():
        pid_str = f"({row['Kp']:.2f},{row['Ki']:.3f},{row['Kd']:.2f})"
        print(f"{row['config']:<22} {pid_str:<20} {row['interval']:<5} "
              f"{row['success_rate']*100:>5.0f}%  "
              f"{row['avg_length']:>8.1f}±{row['std_length']:>5.1f}     "
              f"{row['avg_time']:>6.3f}     "
              f"{row['avg_pareto']:>5.0f}")
    
    print("="*110)
    
    # Find optimal
    best = df.iloc[0]
    print(f"\n{'='*90}")
    print("OPTIMAL CONFIGURATION FOUND".center(90))
    print(f"{'='*90}")
    print(f"  Name: {best['config']}")
    print(f"  PID: Kp={best['Kp']}, Ki={best['Ki']}, Kd={best['Kd']}")
    print(f"  Pareto Interval: {best['interval']}")
    print(f"  Success Rate: {best['success_rate']*100:.0f}%")
    print(f"  Path Length: {best['avg_length']:.2f} ± {best['std_length']:.2f}")
    print(f"  Planning Time: {best['avg_time']:.3f}s")
    print(f"  Pareto Selections: {best['avg_pareto']:.0f}")
    print(f"{'='*90}")
    
    # Compare with baseline
    baseline = df[df['config'] == 'Baseline_NoPareto']
    if len(baseline) > 0:
        baseline = baseline.iloc[0]
        improve_len = (baseline['avg_length'] - best['avg_length']) / baseline['avg_length'] * 100
        improve_time = (baseline['avg_time'] - best['avg_time']) / baseline['avg_time'] * 100
        
        print(f"\nIMPROVEMENT vs NO-PARETO BASELINE:")
        print(f"  Path Length: {improve_len:+.1f}%")
        print(f"  Planning Time: {improve_time:+.1f}%")
        print(f"{'='*90}\n")
    
    return df


if __name__ == "__main__":
    results = fine_tune_optimization()
