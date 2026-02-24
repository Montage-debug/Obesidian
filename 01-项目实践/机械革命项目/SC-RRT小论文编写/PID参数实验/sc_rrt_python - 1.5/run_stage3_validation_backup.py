# -*- coding: utf-8 -*-
"""
Stage 3: Final validation of optimal PID configuration
Run 20 times to get statistical reliability
"""

import sys
from pathlib import Path
import numpy as np
import pandas as pd
from datetime import datetime

current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir / 'src'))

from src.environment import EnvironmentConfig
from src.sc_rrt_basic_pid import SCRRTBasicPID


def run_stage3_validation(kp, ki, kd, pareto_interval=4, num_runs=20):
    """
    Run Stage 3 validation with specified PID parameters
    
    Parameters:
    -----------
    kp : float
        Proportional gain
    ki : float
        Integral gain
    kd : float
        Derivative gain
    pareto_interval : int
        Pareto optimization interval (default: 4 from paper)
    num_runs : int
        Number of validation runs (default: 20)
    
    Returns:
    --------
    pd.DataFrame : Detailed results for each run
    dict : Summary statistics
    """
    
    print("\n" + "="*90)
    print("STAGE 3: FINAL VALIDATION")
    print("="*90)
    print(f"\nConfiguration:")
    print(f"  Kp = {kp:.3f}")
    print(f"  Ki = {ki:.3f}")
    print(f"  Kd = {kd:.3f}")
    print(f"  Pareto Interval = {pareto_interval}")
    print(f"  Number of Runs = {num_runs}")
    print("\n" + "="*90)
    
    # Generate environment (same as Stage 1 & 2)
    env = EnvironmentConfig.generate_2d_environment(
        bounds=[0, 800, 0, 800], 
        num_obstacles=15,
        start_point=np.array([50, 50]), 
        goal_point=np.array([750, 750]),
        radius_range=(10.0, 25.0), 
        seed=42
    )
    
    results = []
    
    for trial in range(1, num_runs + 1):
        print(f"\nRun {trial}/{num_runs}: ", end='', flush=True)
        
        planner = SCRRTBasicPID(
            env=env,
            max_iterations=5000,
            mode='custom_pid',
            Kp=kp, Ki=ki, Kd=kd,
            use_pareto=True,
            pareto_interval=pareto_interval,
            pareto_prob=0.8,
            verbose=False
        )
        
        path, tree, success, metrics = planner.plan()
        
        result = {
            'run_id': trial,
            'success': success,
            'path_length': metrics['path_length'] if success else np.nan,
            'planning_time': metrics['planning_time'],
            'iterations': metrics['iterations'],
            'tree_size': metrics.get('tree_nodes', metrics.get('tree_size', np.nan)),  # 兼容两种字段名
            'kp': kp,
            'ki': ki,
            'kd': kd,
            'pareto_interval': pareto_interval
        }
        
        results.append(result)
        
        if success:
            print(f"✓ Length: {metrics['path_length']:.2f}, Time: {metrics['planning_time']:.3f}s, Iter: {metrics['iterations']}")
        else:
            print(f"✗ Failed after {metrics['iterations']} iterations")
    
    # Convert to DataFrame
    df = pd.DataFrame(results)
    
    # Calculate summary statistics
    successful_runs = df[df['success'] == True]
    
    summary = {
        'total_runs': num_runs,
        'successful_runs': len(successful_runs),
        'success_rate': len(successful_runs) / num_runs,
        'failed_runs': num_runs - len(successful_runs)
    }
    
    if len(successful_runs) > 0:
        summary.update({
            'path_length_mean': successful_runs['path_length'].mean(),
            'path_length_std': successful_runs['path_length'].std(),
            'path_length_min': successful_runs['path_length'].min(),
            'path_length_max': successful_runs['path_length'].max(),
            'path_length_median': successful_runs['path_length'].median(),
            'path_length_q1': successful_runs['path_length'].quantile(0.25),
            'path_length_q3': successful_runs['path_length'].quantile(0.75),
            'planning_time_mean': successful_runs['planning_time'].mean(),
            'planning_time_std': successful_runs['planning_time'].std(),
            'planning_time_min': successful_runs['planning_time'].min(),
            'planning_time_max': successful_runs['planning_time'].max(),
            'iterations_mean': successful_runs['iterations'].mean(),
            'iterations_std': successful_runs['iterations'].std(),
            'tree_size_mean': successful_runs['tree_size'].mean(),
            'tree_size_std': successful_runs['tree_size'].std()
        })
        
        # Calculate 95% confidence interval
        n = len(successful_runs)
        se = summary['path_length_std'] / np.sqrt(n)
        ci_95 = 1.96 * se
        summary['path_length_ci95_lower'] = summary['path_length_mean'] - ci_95
        summary['path_length_ci95_upper'] = summary['path_length_mean'] + ci_95
    
    # Print summary
    print("\n" + "="*90)
    print("VALIDATION SUMMARY")
    print("="*90)
    print(f"\nSuccess Rate: {summary['success_rate']*100:.1f}% ({summary['successful_runs']}/{summary['total_runs']})")
    
    if summary['successful_runs'] > 0:
        print(f"\nPath Length:")
        print(f"  Mean:      {summary['path_length_mean']:.2f}")
        print(f"  Std Dev:   {summary['path_length_std']:.2f}")
        print(f"  Min:       {summary['path_length_min']:.2f}")
        print(f"  Q1:        {summary['path_length_q1']:.2f}")
        print(f"  Median:    {summary['path_length_median']:.2f}")
        print(f"  Q3:        {summary['path_length_q3']:.2f}")
        print(f"  Max:       {summary['path_length_max']:.2f}")
        print(f"  95% CI:    [{summary['path_length_ci95_lower']:.2f}, {summary['path_length_ci95_upper']:.2f}]")
        
        print(f"\nPlanning Time:")
        print(f"  Mean:      {summary['planning_time_mean']:.3f}s")
        print(f"  Std Dev:   {summary['planning_time_std']:.3f}s")
        print(f"  Min:       {summary['planning_time_min']:.3f}s")
        print(f"  Max:       {summary['planning_time_max']:.3f}s")
        
        print(f"\nIterations:")
        print(f"  Mean:      {summary['iterations_mean']:.0f}")
        print(f"  Std Dev:   {summary['iterations_std']:.0f}")
        
        print(f"\nTree Size:")
        print(f"  Mean:      {summary['tree_size_mean']:.0f}")
        print(f"  Std Dev:   {summary['tree_size_std']:.0f}")
    else:
        print("\n⚠️  All runs failed!")
    
    print("="*90)
    
    return df, summary


def save_stage3_results(df, summary, kp, ki, kd):
    """Save Stage 3 results to files"""
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results_dir = Path("results")
    results_dir.mkdir(exist_ok=True)
    
    # Save detailed results CSV
    csv_file = results_dir / f"stage3_validation_{timestamp}.csv"
    df.to_csv(csv_file, index=False)
    print(f"\n✓ Detailed results saved: {csv_file}")
    
    # Save summary statistics
    summary_file = results_dir / f"stage3_summary_{timestamp}.txt"
    with open(summary_file, 'w', encoding='utf-8') as f:
        f.write("="*70 + "\n")
        f.write("STAGE 3: FINAL VALIDATION SUMMARY\n")
        f.write("="*70 + "\n\n")
        
        f.write("Configuration:\n")
        f.write(f"  Kp = {kp:.3f}\n")
        f.write(f"  Ki = {ki:.3f}\n")
        f.write(f"  Kd = {kd:.3f}\n")
        f.write(f"  Pareto Interval = {summary.get('pareto_interval', 4)}\n\n")
        
        f.write(f"Success Rate: {summary['success_rate']*100:.1f}% ")
        f.write(f"({summary['successful_runs']}/{summary['total_runs']})\n\n")
        
        if summary['successful_runs'] > 0:
            f.write("Path Length:\n")
            f.write(f"  Mean:      {summary['path_length_mean']:.2f}\n")
            f.write(f"  Std Dev:   {summary['path_length_std']:.2f}\n")
            f.write(f"  Min:       {summary['path_length_min']:.2f}\n")
            f.write(f"  Q1:        {summary['path_length_q1']:.2f}\n")
            f.write(f"  Median:    {summary['path_length_median']:.2f}\n")
            f.write(f"  Q3:        {summary['path_length_q3']:.2f}\n")
            f.write(f"  Max:       {summary['path_length_max']:.2f}\n")
            f.write(f"  95% CI:    [{summary['path_length_ci95_lower']:.2f}, ")
            f.write(f"{summary['path_length_ci95_upper']:.2f}]\n\n")
            
            f.write("Planning Time:\n")
            f.write(f"  Mean:      {summary['planning_time_mean']:.3f}s\n")
            f.write(f"  Std Dev:   {summary['planning_time_std']:.3f}s\n")
            f.write(f"  Range:     [{summary['planning_time_min']:.3f}, ")
            f.write(f"{summary['planning_time_max']:.3f}]s\n\n")
            
            f.write("Iterations:\n")
            f.write(f"  Mean:      {summary['iterations_mean']:.0f}\n")
            f.write(f"  Std Dev:   {summary['iterations_std']:.0f}\n\n")
            
            f.write("Tree Size:\n")
            f.write(f"  Mean:      {summary['tree_size_mean']:.0f}\n")
            f.write(f"  Std Dev:   {summary['tree_size_std']:.0f}\n")
    
    print(f"✓ Summary saved: {summary_file}")
    
    # Save summary as CSV for easy analysis
    summary_csv = results_dir / f"stage3_summary_{timestamp}.csv"
    pd.DataFrame([summary]).to_csv(summary_csv, index=False)
    print(f"✓ Summary CSV saved: {summary_csv}")
    
    return csv_file, summary_file


def main():
    """Main execution"""
    
    # Read optimal configuration from Stage 2 results
    stage2_file = "results/stage2_fine_20260209_230704.csv"
    
    if Path(stage2_file).exists():
        print(f"\nLoading Stage 2 results from: {stage2_file}")
        stage2_df = pd.read_csv(stage2_file)
        
        # Get best configuration
        best = stage2_df.iloc[0]
        kp = best['Kp']
        ki = best['Ki']
        kd = best['Kd']
        
        print(f"\nBest configuration from Stage 2:")
        print(f"  Kp = {kp:.3f}")
        print(f"  Ki = {ki:.3f}")
        print(f"  Kd = {kd:.3f}")
        print(f"  Stage 2 avg path: {best['avg_length']:.2f} ± {best['std_length']:.2f}")
    else:
        print(f"\n⚠️  Stage 2 file not found: {stage2_file}")
        print("Using default optimal configuration:")
        kp = 0.26
        ki = 0.024
        kd = 0.06
        print(f"  Kp = {kp}")
        print(f"  Ki = {ki}")
        print(f"  Kd = {kd}")
    
    # Run Stage 3 validation
    start_time = datetime.now()
    df, summary = run_stage3_validation(kp, ki, kd, pareto_interval=4, num_runs=20)
    elapsed = datetime.now() - start_time
    
    # Save results
    save_stage3_results(df, summary, kp, ki, kd)
    
    print(f"\n{'='*90}")
    print(f"Total execution time: {elapsed}")
    print(f"{'='*90}\n")


if __name__ == "__main__":
    main()
