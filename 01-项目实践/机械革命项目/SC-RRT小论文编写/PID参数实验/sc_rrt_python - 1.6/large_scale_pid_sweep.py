# -*- coding: utf-8 -*-
"""
Large-scale PID Parameter Sweep with Fixed Pareto Interval=4
Stage 1: Coarse grid search
Stage 2: Fine-tuning around best region
"""

import sys
from pathlib import Path
import numpy as np
import pandas as pd
from datetime import datetime
import itertools

current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir / 'src'))

from src.environment import EnvironmentConfig
from src.sc_rrt_basic_pid import SCRRTBasicPID


# Parameter ranges as specified
KP_RANGE = [
    0.05, 0.10,
    0.14, 0.16, 0.18, 0.19, 0.20, 0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.28, 0.30,
    0.35, 0.40, 0.45
]

KI_RANGE = [
    0.00, 0.01,
    0.018, 0.020, 0.022, 0.024, 0.026, 0.028, 0.030, 0.032, 0.034, 0.035, 0.038, 0.040,
    0.05, 0.06, 0.07
]

KD_RANGE = [
    0.03,
    0.055, 0.060, 0.065, 0.070, 0.075, 0.080, 0.085, 0.090, 0.095, 0.100, 0.105,
    0.12, 0.15, 0.18
]

PARETO_INTERVAL = 4  # Fixed as per paper's optimal


def stage1_coarse_search():
    """Stage 1: Coarse grid search on boundary and key points"""
    
    print("\n" + "="*90)
    print("STAGE 1: COARSE GRID SEARCH")
    print("="*90)
    
    # Select representative points for coarse search
    kp_coarse = [KP_RANGE[0], KP_RANGE[3], KP_RANGE[6], KP_RANGE[9], KP_RANGE[13], KP_RANGE[-1]]  # 6 points
    ki_coarse = [KI_RANGE[0], KI_RANGE[2], KI_RANGE[6], KI_RANGE[10], KI_RANGE[13], KI_RANGE[-1]]  # 6 points
    kd_coarse = [KD_RANGE[0], KD_RANGE[3], KD_RANGE[6], KD_RANGE[9], KD_RANGE[-1]]  # 5 points
    
    total_configs = len(kp_coarse) * len(ki_coarse) * len(kd_coarse)
    print(f"Testing {total_configs} configurations (6×6×5)")
    print(f"Kp: {kp_coarse}")
    print(f"Ki: {ki_coarse}")
    print(f"Kd: {kd_coarse}")
    print(f"Pareto Interval: {PARETO_INTERVAL} (fixed)")
    
    env = EnvironmentConfig.generate_3d_environment(
        bounds=[0, 1500, 0, 1500, 0, 1500], num_obstacles=400,
        start_point=np.array([100, 100, 100]), goal_point=np.array([1400, 1400, 1400]),
        radius_range=(15.0, 30.0), seed=42
    )
    
    results = []
    config_num = 0
    
    for kp in kp_coarse:
        for ki in ki_coarse:
            for kd in kd_coarse:
                config_num += 1
                print(f"\n[{config_num}/{total_configs}] Kp={kp:.2f}, Ki={ki:.3f}, Kd={kd:.2f}", end=' ')
                
                # Run 7 trials per configuration for reliable statistics
                run_data = []
                for trial in range(7):
                    planner = SCRRTBasicPID(
                        env=env, max_iterations=5000, mode='custom_pid',
                        Kp=kp, Ki=ki, Kd=kd,
                        use_pareto=True, pareto_interval=PARETO_INTERVAL,
                        pareto_prob=0.8, verbose=False
                    )
                    
                    path, tree, success, metrics = planner.plan()
                    run_data.append({
                        'success': success,
                        'length': metrics['path_length'] if success else np.inf,
                        'time': metrics['planning_time']
                    })
                    
                    if success:
                        print(f"{metrics['path_length']:.0f}", end=' ')
                    else:
                        print("FAIL", end=' ')
                
                # Calculate statistics
                success_rate = sum(r['success'] for r in run_data) / len(run_data)
                
                if success_rate > 0:
                    successful = [r for r in run_data if r['success']]
                    avg_len = np.mean([r['length'] for r in successful])
                    std_len = np.std([r['length'] for r in successful])
                    avg_time = np.mean([r['time'] for r in successful])
                    
                    results.append({
                        'Kp': kp, 'Ki': ki, 'Kd': kd,
                        'success_rate': success_rate,
                        'avg_length': avg_len,
                        'std_length': std_len,
                        'avg_time': avg_time
                    })
                    
                    print(f"-> Avg: {avg_len:.1f}±{std_len:.1f}")
                else:
                    print("-> All failed")
    
    df = pd.DataFrame(results)
    df = df.sort_values('avg_length')
    
    print("\n" + "="*90)
    print("STAGE 1 TOP 10 RESULTS")
    print("="*90)
    print(f"{'Rank':<6} {'Kp':<7} {'Ki':<8} {'Kd':<7} {'Success%':<10} {'Avg Length':<15} {'Avg Time(s)':<12}")
    print("-"*90)
    
    for idx, row in df.head(10).iterrows():
        print(f"{df.index.get_loc(idx)+1:<6} {row['Kp']:<7.2f} {row['Ki']:<8.3f} {row['Kd']:<7.2f} "
              f"{row['success_rate']*100:>6.0f}%     {row['avg_length']:>8.1f}±{row['std_length']:>4.1f}    "
              f"{row['avg_time']:>8.3f}")
    
    print("="*90)
    
    return df


def stage2_fine_tuning(stage1_results):
    """Stage 2: Fine-tune around best regions from Stage 1"""
    
    print("\n" + "="*90)
    print("STAGE 2: FINE-TUNING AROUND BEST REGION")
    print("="*90)
    
    # Get top 3 configurations
    top3 = stage1_results.head(3)
    
    print("\nBest regions from Stage 1:")
    for idx, row in top3.iterrows():
        print(f"  Kp={row['Kp']:.2f}, Ki={row['Ki']:.3f}, Kd={row['Kd']:.2f} -> {row['avg_length']:.1f}")
    
    # Find best Kp, Ki, Kd ranges
    best_kp = top3['Kp'].iloc[0]
    best_ki = top3['Ki'].iloc[0]
    best_kd = top3['Kd'].iloc[0]
    
    # Select fine-grained values around best
    kp_fine = [kp for kp in KP_RANGE if abs(kp - best_kp) <= 0.10]
    ki_fine = [ki for ki in KI_RANGE if abs(ki - best_ki) <= 0.015]
    kd_fine = [kd for kd in KD_RANGE if abs(kd - best_kd) <= 0.025]
    
    print(f"\nFine-tuning ranges:")
    print(f"Kp ({len(kp_fine)}): {kp_fine}")
    print(f"Ki ({len(ki_fine)}): {ki_fine}")
    print(f"Kd ({len(kd_fine)}): {kd_fine}")
    
    total_configs = len(kp_fine) * len(ki_fine) * len(kd_fine)
    print(f"\nTesting {total_configs} configurations")
    
    env = EnvironmentConfig.generate_3d_environment(
        bounds=[0, 1500, 0, 1500, 0, 1500], num_obstacles=400,
        start_point=np.array([100, 100, 100]), goal_point=np.array([1400, 1400, 1400]),
        radius_range=(15.0, 30.0), seed=42
    )
    
    results = []
    config_num = 0
    
    for kp in kp_fine:
        for ki in ki_fine:
            for kd in kd_fine:
                config_num += 1
                print(f"\n[{config_num}/{total_configs}] Kp={kp:.2f}, Ki={ki:.3f}, Kd={kd:.2f}", end=' ')
                
                # Run 10 trials for high-precision fine-tuning
                run_data = []
                for trial in range(10):
                    planner = SCRRTBasicPID(
                        env=env, max_iterations=5000, mode='custom_pid',
                        Kp=kp, Ki=ki, Kd=kd,
                        use_pareto=True, pareto_interval=PARETO_INTERVAL,
                        pareto_prob=0.8, verbose=False
                    )
                    
                    path, tree, success, metrics = planner.plan()
                    run_data.append({
                        'success': success,
                        'length': metrics['path_length'] if success else np.inf,
                        'time': metrics['planning_time']
                    })
                    
                    if success:
                        print(f"{metrics['path_length']:.0f}", end=' ')
                    else:
                        print("F", end=' ')
                
                success_rate = sum(r['success'] for r in run_data) / len(run_data)
                
                if success_rate > 0:
                    successful = [r for r in run_data if r['success']]
                    avg_len = np.mean([r['length'] for r in successful])
                    std_len = np.std([r['length'] for r in successful])
                    avg_time = np.mean([r['time'] for r in successful])
                    
                    results.append({
                        'Kp': kp, 'Ki': ki, 'Kd': kd,
                        'success_rate': success_rate,
                        'avg_length': avg_len,
                        'std_length': std_len,
                        'avg_time': avg_time
                    })
                    
                    print(f"-> {avg_len:.1f}±{std_len:.1f}")
                else:
                    print("-> Failed")
    
    df = pd.DataFrame(results)
    df = df.sort_values('avg_length')
    
    print("\n" + "="*90)
    print("STAGE 2 TOP 10 RESULTS")
    print("="*90)
    print(f"{'Rank':<6} {'Kp':<7} {'Ki':<8} {'Kd':<7} {'Success%':<10} {'Avg Length':<15} {'Avg Time(s)':<12}")
    print("-"*90)
    
    for idx, row in df.head(10).iterrows():
        print(f"{df.index.get_loc(idx)+1:<6} {row['Kp']:<7.2f} {row['Ki']:<8.3f} {row['Kd']:<7.2f} "
              f"{row['success_rate']*100:>6.0f}%     {row['avg_length']:>8.1f}±{row['std_length']:>4.1f}    "
              f"{row['avg_time']:>8.3f}")
    
    print("="*90)
    
    return df


def stage3_final_validation(stage2_results):
    """Stage 3: Final validation of best configuration with 30 runs"""
    
    print("\n" + "="*90)
    print("STAGE 3: FINAL VALIDATION (30 RUNS)")
    print("="*90)
    
    best = stage2_results.iloc[0]
    
    print(f"\nBest Configuration:")
    print(f"  Kp = {best['Kp']:.3f}")
    print(f"  Ki = {best['Ki']:.3f}")
    print(f"  Kd = {best['Kd']:.3f}")
    print(f"  Pareto Interval = {PARETO_INTERVAL}")
    print(f"\nStage 2 Performance: {best['avg_length']:.2f} ± {best['std_length']:.2f}")
    print(f"\nRunning 20 validation trials...")
    
    env = EnvironmentConfig.generate_2d_environment(
        bounds=[0, 800, 0, 800], num_obstacles=15,
        start_point=np.array([50, 50]), goal_point=np.array([750, 750]),
        radius_range=(10.0, 25.0), seed=42
    )
    
    run_data = []
    for trial in range(30):
        print(f"\n  Trial {trial+1}/30: ", end='', flush=True)
        
        planner = SCRRTBasicPID(
            env=env, max_iterations=5000, mode='custom_pid',
            Kp=best['Kp'], Ki=best['Ki'], Kd=best['Kd'],
            use_pareto=True, pareto_interval=PARETO_INTERVAL,
            pareto_prob=0.8, verbose=False
        )
        
        path, tree, success, metrics = planner.plan()
        
        run_data.append({
            'success': success,
            'length': metrics['path_length'] if success else np.inf,
            'time': metrics['planning_time'],
            'iterations': metrics['iterations']
        })
        
        if success:
            print(f"Success - Length: {metrics['path_length']:.1f}, Time: {metrics['planning_time']:.2f}s")
        else:
            print("Failed")
    
    # Final statistics
    success_rate = sum(r['success'] for r in run_data) / len(run_data)
    
    print("\n" + "="*90)
    print("FINAL VALIDATION RESULTS")
    print("="*90)
    
    if success_rate > 0:
        successful = [r for r in run_data if r['success']]
        lengths = [r['length'] for r in successful]
        times = [r['time'] for r in successful]
        
        print(f"\nSuccess Rate: {success_rate*100:.1f}%")
        print(f"\nPath Length:")
        print(f"  Mean: {np.mean(lengths):.2f}")
        print(f"  Std:  {np.std(lengths):.2f}")
        print(f"  Min:  {np.min(lengths):.2f}")
        print(f"  Max:  {np.max(lengths):.2f}")
        print(f"  Median: {np.median(lengths):.2f}")
        
        print(f"\nPlanning Time:")
        print(f"  Mean: {np.mean(times):.3f}s")
        print(f"  Std:  {np.std(times):.3f}s")
        print(f"  Min:  {np.min(times):.3f}s")
        print(f"  Max:  {np.max(times):.3f}s")
        
        print(f"\nIterations:")
        print(f"  Mean: {np.mean([r['iterations'] for r in successful]):.0f}")
    
    print("="*90)
    
    return run_data


def main():
    """Main execution"""
    
    print("\n" + "="*90)
    print("LARGE-SCALE PID PARAMETER SWEEP")
    print("="*90)
    print(f"\nTotal parameter space: {len(KP_RANGE)}×{len(KI_RANGE)}×{len(KD_RANGE)} = {len(KP_RANGE)*len(KI_RANGE)*len(KD_RANGE)} combinations")
    print(f"Pareto Interval: {PARETO_INTERVAL} (fixed)")
    print(f"\nStrategy: 3-stage intelligent search")
    print(f"  Stage 1: Coarse grid (6×6×5 = 180 configs, 7 runs each)")
    print(f"  Stage 2: Fine-tuning around best region (10 runs each)")
    print(f"  Stage 3: Final validation (30 runs)")
    
    start_time = datetime.now()
    
    # Stage 1
    stage1_results = stage1_coarse_search()
    
    # Save Stage 1 results
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    stage1_results.to_csv(f"results/stage1_coarse_{timestamp}.csv", index=False)
    print(f"\nStage 1 results saved to: results/stage1_coarse_{timestamp}.csv")
    
    # Stage 2
    stage2_results = stage2_fine_tuning(stage1_results)
    
    # Save Stage 2 results
    stage2_results.to_csv(f"results/stage2_fine_{timestamp}.csv", index=False)
    print(f"\nStage 2 results saved to: results/stage2_fine_{timestamp}.csv")
    
    # Stage 3
    validation_data = stage3_final_validation(stage2_results)
    
    # Save final results
    best = stage2_results.iloc[0]
    with open(f"results/final_optimal_config_{timestamp}.txt", 'w') as f:
        f.write("OPTIMAL PID CONFIGURATION\n")
        f.write("="*50 + "\n\n")
        f.write(f"Kp = {best['Kp']:.3f}\n")
        f.write(f"Ki = {best['Ki']:.3f}\n")
        f.write(f"Kd = {best['Kd']:.3f}\n")
        f.write(f"Pareto Interval = {PARETO_INTERVAL}\n")
        f.write(f"\nValidation (20 runs):\n")
        f.write(f"  Success Rate: {sum(r['success'] for r in validation_data)/len(validation_data)*100:.1f}%\n")
        successful = [r for r in validation_data if r['success']]
        if successful:
            f.write(f"  Path Length: {np.mean([r['length'] for r in successful]):.2f} ± {np.std([r['length'] for r in successful]):.2f}\n")
            f.write(f"  Planning Time: {np.mean([r['time'] for r in successful]):.3f} ± {np.std([r['time'] for r in successful]):.3f}s\n")
    
    print(f"\nFinal configuration saved to: results/final_optimal_config_{timestamp}.txt")
    
    elapsed = datetime.now() - start_time
    print(f"\n" + "="*90)
    print(f"TOTAL TIME: {elapsed}")
    print("="*90)


if __name__ == "__main__":
    main()
