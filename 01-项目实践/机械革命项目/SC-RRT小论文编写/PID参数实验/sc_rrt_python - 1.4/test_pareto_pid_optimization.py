# -*- coding: utf-8 -*-
"""
SC-RRT: Pareto Selection Strategy + PID Optimization Experiment
Testing different Pareto configurations with PID control
"""

import sys
from pathlib import Path
import numpy as np
import pandas as pd
from datetime import datetime

current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))
sys.path.insert(0, str(current_dir / 'src'))

from src.environment import EnvironmentConfig
from src.sc_rrt_basic_pid import SCRRTBasicPID


def print_header(text):
    """Print formatted header"""
    print("\n" + "=" * 70)
    print(text.center(70))
    print("=" * 70)


def run_pareto_pid_experiment():
    """
    Test Pareto selection strategy with different PID configurations
    Goal: Find optimal combination of Pareto interval and PID parameters
    """
    print_header("Pareto Selection + PID Optimization Experiment")
    
    # Test environment
    env_2d = EnvironmentConfig.generate_2d_environment(
        bounds=[0, 800, 0, 800],
        num_obstacles=15,
        start_point=np.array([50, 50]),
        goal_point=np.array([750, 750]),
        radius_range=(10.0, 25.0),
        seed=42
    )
    
    # Test configurations
    test_configs = [
        # Baseline
        {'name': 'Baseline_No_Pareto', 'Kp': 0.25, 'Ki': 0.04, 'Kd': 0.10, 
         'use_pareto': False, 'pareto_interval': 100, 'pareto_prob': 0.8},
        
        # Current optimal PID with different Pareto intervals
        {'name': 'PID_Pareto_50', 'Kp': 0.25, 'Ki': 0.04, 'Kd': 0.10,
         'use_pareto': True, 'pareto_interval': 50, 'pareto_prob': 0.8},
        {'name': 'PID_Pareto_100', 'Kp': 0.25, 'Ki': 0.04, 'Kd': 0.10,
         'use_pareto': True, 'pareto_interval': 100, 'pareto_prob': 0.8},
        {'name': 'PID_Pareto_150', 'Kp': 0.25, 'Ki': 0.04, 'Kd': 0.10,
         'use_pareto': True, 'pareto_interval': 150, 'pareto_prob': 0.8},
        
        # Higher PID gains with Pareto
        {'name': 'HighPID_Pareto_100', 'Kp': 0.30, 'Ki': 0.05, 'Kd': 0.12,
         'use_pareto': True, 'pareto_interval': 100, 'pareto_prob': 0.8},
        
        # Lower PID gains with Pareto
        {'name': 'LowPID_Pareto_100', 'Kp': 0.20, 'Ki': 0.03, 'Kd': 0.08,
         'use_pareto': True, 'pareto_interval': 100, 'pareto_prob': 0.8},
        
        # Optimal PID + aggressive Pareto
        {'name': 'PID_Pareto_75', 'Kp': 0.25, 'Ki': 0.04, 'Kd': 0.10,
         'use_pareto': True, 'pareto_interval': 75, 'pareto_prob': 0.8},
    ]
    
    results = []
    
    for config in test_configs:
        print(f"\n{'='*70}")
        print(f"Testing: {config['name']}")
        print(f"  PID: Kp={config['Kp']}, Ki={config['Ki']}, Kd={config['Kd']}")
        print(f"  Pareto: enabled={config['use_pareto']}, interval={config['pareto_interval']}")
        print(f"{'='*70}")
        
        # Run multiple trials
        run_results = []
        for run_id in range(5):  # 5 runs per configuration
            print(f"\n  Run {run_id+1}/5... ", end='', flush=True)
            
            planner = SCRRTBasicPID(
                env=env_2d,
                max_iterations=5000,
                mode='custom_pid',
                Kp=config['Kp'],
                Ki=config['Ki'],
                Kd=config['Kd'],
                use_pareto=config['use_pareto'],
                pareto_interval=config['pareto_interval'],
                pareto_prob=config['pareto_prob'],
                verbose=False
            )
            
            path, tree, success, metrics = planner.plan()
            
            run_results.append({
                'success': success,
                'path_length': metrics['path_length'] if success else np.inf,
                'planning_time': metrics['planning_time'],
                'iterations': metrics['iterations'],
                'pareto_selections': len(planner.pareto_selections),
                'first_solution_iter': metrics.get('first_solution_iter', np.inf),
                'first_solution_time': metrics.get('first_solution_time', np.inf),
            })
            
            print(f"{'Success' if success else 'Failed'} (Length={metrics['path_length']:.1f})" if success 
                  else f"Failed")
        
        # Calculate statistics
        success_count = sum(1 for r in run_results if r['success'])
        success_rate = success_count / len(run_results)
        
        if success_count > 0:
            successful_runs = [r for r in run_results if r['success']]
            avg_path_length = np.mean([r['path_length'] for r in successful_runs])
            std_path_length = np.std([r['path_length'] for r in successful_runs])
            avg_time = np.mean([r['planning_time'] for r in successful_runs])
            avg_iterations = np.mean([r['iterations'] for r in successful_runs])
            avg_pareto_sel = np.mean([r['pareto_selections'] for r in successful_runs])
            avg_first_iter = np.mean([r['first_solution_iter'] for r in successful_runs])
            avg_first_time = np.mean([r['first_solution_time'] for r in successful_runs])
        else:
            avg_path_length = np.inf
            std_path_length = np.inf
            avg_time = np.mean([r['planning_time'] for r in run_results])
            avg_iterations = np.mean([r['iterations'] for r in run_results])
            avg_pareto_sel = 0
            avg_first_iter = np.inf
            avg_first_time = np.inf
        
        results.append({
            'config': config['name'],
            'Kp': config['Kp'],
            'Ki': config['Ki'],
            'Kd': config['Kd'],
            'use_pareto': config['use_pareto'],
            'pareto_interval': config['pareto_interval'],
            'success_rate': success_rate,
            'avg_path_length': avg_path_length,
            'std_path_length': std_path_length,
            'avg_time': avg_time,
            'avg_iterations': avg_iterations,
            'avg_pareto_selections': avg_pareto_sel,
            'avg_first_solution_iter': avg_first_iter,
            'avg_first_solution_time': avg_first_time,
        })
        
        print(f"\n  Results:")
        print(f"    Success Rate: {success_rate*100:.1f}%")
        if success_rate > 0:
            print(f"    Avg Path Length: {avg_path_length:.2f} (±{std_path_length:.2f})")
            print(f"    Avg Time: {avg_time:.3f}s")
            print(f"    Avg Iterations: {avg_iterations:.0f}")
            print(f"    Avg Pareto Selections: {avg_pareto_sel:.1f}")
            print(f"    First Solution: iter {avg_first_iter:.0f} ({avg_first_time:.3f}s)")
    
    # Print summary table
    print_header("EXPERIMENT SUMMARY")
    df = pd.DataFrame(results)
    
    print("\n" + "=" * 120)
    print(f"{'Configuration':<25} {'Success':<10} {'Path Length':<20} {'Time(s)':<12} {'Iter':<8} {'Pareto#':<10}")
    print("=" * 120)
    
    for _, row in df.iterrows():
        if row['success_rate'] > 0:
            print(f"{row['config']:<25} {row['success_rate']*100:>6.1f}%    "
                  f"{row['avg_path_length']:>10.2f} (±{row['std_path_length']:>5.2f})  "
                  f"{row['avg_time']:>8.3f}     "
                  f"{row['avg_iterations']:>6.0f}   "
                  f"{row['avg_pareto_selections']:>6.1f}")
        else:
            print(f"{row['config']:<25} {row['success_rate']*100:>6.1f}%    "
                  f"{'N/A':<20}  "
                  f"{row['avg_time']:>8.3f}     "
                  f"{row['avg_iterations']:>6.0f}   "
                  f"{'N/A':<10}")
    
    print("=" * 120)
    
    # Find best configuration
    successful_configs = df[df['success_rate'] == 1.0]
    if len(successful_configs) > 0:
        # Sort by path length (primary) and time (secondary)
        successful_configs = successful_configs.sort_values(['avg_path_length', 'avg_time'])
        best = successful_configs.iloc[0]
        
        print(f"\n{'='*70}")
        print("BEST CONFIGURATION FOUND".center(70))
        print(f"{'='*70}")
        print(f"  Name: {best['config']}")
        print(f"  PID Parameters: Kp={best['Kp']}, Ki={best['Ki']}, Kd={best['Kd']}")
        print(f"  Pareto: enabled={best['use_pareto']}, interval={best['pareto_interval']}")
        print(f"  Success Rate: 100%")
        print(f"  Path Length: {best['avg_path_length']:.2f} (±{best['std_path_length']:.2f})")
        print(f"  Planning Time: {best['avg_time']:.3f}s")
        print(f"  Iterations: {best['avg_iterations']:.0f}")
        print(f"  Pareto Selections: {best['avg_pareto_selections']:.1f}")
        print(f"{'='*70}")
        
        # Calculate improvement over baseline
        baseline = df[df['config'] == 'Baseline_No_Pareto'].iloc[0]
        if baseline['success_rate'] > 0:
            path_improvement = (baseline['avg_path_length'] - best['avg_path_length']) / baseline['avg_path_length'] * 100
            time_improvement = (baseline['avg_time'] - best['avg_time']) / baseline['avg_time'] * 100
            
            print(f"\nIMPROVEMENT vs BASELINE:")
            print(f"  Path Length: {path_improvement:+.1f}%")
            print(f"  Planning Time: {time_improvement:+.1f}%")
            print(f"{'='*70}")
    
    # Save results
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_file = f"results/pareto_pid_experiment_{timestamp}.csv"
    df.to_csv(csv_file, index=False)
    print(f"\nResults saved to: {csv_file}")
    
    return df


if __name__ == "__main__":
    print_header("SC-RRT: Pareto + PID Optimization")
    print("\nObjective: Find optimal combination of:")
    print("  1. PID parameters (Kp, Ki, Kd)")
    print("  2. Pareto selection interval")
    print("\nTest Environment: 2D (800x800) with 15 obstacles")
    print("Trials per configuration: 5 runs")
    
    results = run_pareto_pid_experiment()
    
    print("\n" + "="*70)
    print("EXPERIMENT COMPLETED".center(70))
    print("="*70)
