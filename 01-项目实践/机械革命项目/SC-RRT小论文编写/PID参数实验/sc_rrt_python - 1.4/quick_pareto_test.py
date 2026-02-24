# -*- coding: utf-8 -*-
"""
Quick Pareto + PID Test - Find optimal configuration faster
"""

import sys
from pathlib import Path
import numpy as np

current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir / 'src'))

from src.environment import EnvironmentConfig
from src.sc_rrt_basic_pid import SCRRTBasicPID


def quick_test():
    env = EnvironmentConfig.generate_2d_environment(
        bounds=[0, 800, 0, 800], num_obstacles=15,
        start_point=np.array([50, 50]), goal_point=np.array([750, 750]),
        radius_range=(10.0, 25.0), seed=42
    )
    
    configs = [
        {'name': 'No_Pareto', 'pareto': False, 'interval': 0},
        {'name': 'Pareto_75', 'pareto': True, 'interval': 75},
        {'name': 'Pareto_100', 'pareto': True, 'interval': 100},
        {'name': 'Pareto_125', 'pareto': True, 'interval': 125},
    ]
    
    print("\n" + "="*80)
    print("QUICK TEST: Pareto Interval Optimization (Kp=0.25, Ki=0.04, Kd=0.10)")
    print("="*80)
    
    for cfg in configs:
        results = []
        print(f"\n[{cfg['name']}] ", end='', flush=True)
        
        for i in range(3):
            planner = SCRRTBasicPID(
                env=env, max_iterations=5000, mode='custom_pid',
                Kp=0.25, Ki=0.04, Kd=0.10,
                use_pareto=cfg['pareto'],
                pareto_interval=cfg['interval'],
                pareto_prob=0.8, verbose=False
            )
            
            path, tree, success, metrics = planner.plan()
            results.append({
                'success': success,
                'length': metrics['path_length'] if success else np.inf,
                'time': metrics['planning_time'],
                'iter': metrics['iterations'],
                'pareto_count': len(planner.pareto_selections)
            })
            print(f"{metrics['path_length']:.0f} " if success else "FAIL ", end='', flush=True)
        
        success_rate = sum(r['success'] for r in results) / 3
        if success_rate > 0:
            successful = [r for r in results if r['success']]
            avg_len = np.mean([r['length'] for r in successful])
            avg_time = np.mean([r['time'] for r in successful])
            avg_pareto = np.mean([r['pareto_count'] for r in successful])
            print(f"-> Avg: {avg_len:.1f}, Time: {avg_time:.2f}s, Pareto: {avg_pareto:.0f}")
        else:
            print("-> All failed")
    
    print("\n" + "="*80)


if __name__ == "__main__":
    quick_test()
