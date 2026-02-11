"""
SC-RRT PID Parameter Optimization Experiment
SC-RRT PID参数优化实验 - SCI论文级

Usage:
    Quick Test (15-20 min):  python run_local_pid_experiment.py --quick
    Full Experiment (1-2h):  python run_local_pid_experiment.py
"""

import sys
from pathlib import Path
import argparse
import warnings
warnings.filterwarnings('ignore')

# 添加src目录到Python路径
current_dir = Path(__file__).parent
sys.path.insert(0, str(current_dir))
sys.path.insert(0, str(current_dir / 'src'))

# 导入必要的模块
import numpy as np
import pandas as pd
from datetime import datetime

from src.environment import EnvironmentConfig
from src.sc_rrt_basic_pid import SCRRTBasicPID
from experiments.run_pid_experiment import PIDExperiment
from utils.analyze_results import ResultAnalyzer


def print_header(text):
    """打印格式化的标题"""
    print("\n" + "=" * 70)
    print(text.center(70))
    print("=" * 70)


def run_baseline_experiment():
    """运行PID有效性验证实验（证明PID对双向超椭球体约束采样的有效性）"""
    print_header("PID EFFECTIVENESS VALIDATION - Sampling Behavior Analysis")
    print("\n" + "="*70)
    print("核心创新：PID控制器 + 双向超椭球体约束采样")
    print("="*70)
    print("\n【实验组设计 - V4高难度配置】")
    print("  No_PID      : 无PID对照（纯双向RRT，不使用椭球约束）")
    print("  Low_PID     : 欠调PID（Kp=0.15，探索不足）")
    print("  Optimal_PID : 最优PID（Kp=0.30, Ki=0.05, Kd=0.12）")
    print("  High_PID    : 过调PID（Kp=0.50，过度控制）")
    print("\n【三大核心指标】（直接服务于PID有效性证明）")
    print("  1️⃣  有效采样比例(ESR)          - 证明PID引导采样进入约束区域")
    print("  2️⃣  达到固定成功率所需采样量    - 证明PID提高单位采样价值")
    print("  3️⃣  椭球体收缩稳定性           - 证明PID控制约束机制可控可用")
    print("\n【预期结果 - 高密度障碍物环境】")
    print("  成功率对比:")
    print("    • No_PID: 25-35% (无引导，随机搜索效率低)")
    print("    • Optimal_PID: 50-65% (PID优化引导，成功率提升2倍)")
    print("  效率指标:")
    print("    • ESR: No_PID=0% vs Optimal_PID~45-60%")
    print("    • Sample-to-Success: No_PID~800次 vs Optimal_PID~400次")
    print("\n【实验配置】")
    print("  配置数: 4 组")
    print("  场景数: 2 个（二维225@R25-45 + 三维400@R45-70最佳）")
    print("  重复次数: 30 次/配置（高统计置信度）")
    print("  迭代限制: 2D=1200, 3D=1500 (三维最佳配置：100%放置，9.5%占用)")
    print("  总运行数: 4 × 2 × 30 = 240 次")
    print("  预计耗时: 20-26分钟")
    print("="*70)
    
    # 创建实验对象
    experiment = PIDExperiment(
        quick_test=True,
        baseline_mode=True,
        output_dir='results'
    )
    
    # 运行实验
    print("\n开始PID有效性验证实验...")
    results_df = experiment.run_experiment()
    
    return results_df, experiment


def run_refined_search():
    """运行参数精细化搜索（双区间策略）"""
    print_header("REFINED SEARCH MODE - Dual-Zone Strategy")
    print("\nExperiment Configuration:")
    print("  Based on P4(Kp=0.30) optimal result (45% success rate)")
    print("  - Dense zone: 7 configs around Kp=0.30 (0.26-0.38)")
    print("  - Verify zone: 5 configs around Kp=0.20 (0.15-0.25)")
    print("  - Control group: 2 extreme configs")
    print("  - 2 scenarios (2D-200obs, 3D-500obs)")
    print("  - 10 repeats per group")
    print("  - Total runs: 14 x 2 x 10 = 280")
    print("  - Estimated time: 20-25 minutes")
    
    # 创建实验对象
    experiment = PIDExperiment(
        quick_test=True,  # 使用快速模式的重复次数
        use_refined_search=True,  # 启用参数加密搜索
        output_dir='results'
    )
    
    # 运行实验
    print("\nStarting refined search experiment...")
    results_df = experiment.run_experiment()
    
    return results_df, experiment


def run_fine_tuning():
    """运行三参数精细调优（探索Kp×Ki×Kd三维空间）"""
    print_header("FINE-TUNING MODE - 3D Parameter Space Exploration")
    print("\nExperiment Configuration:")
    print("  Based on Verify_Kp0.20 optimal result (35% success rate)")
    print("  - Core group: 6 configs exploring Ki×Kd space @ Kp=0.20")
    print("  - Cross-validation: 3 configs testing Kp/Ki/Kd sensitivity")
    print("  - Extreme boundary: 3 configs validating parameter limits")
    print("  - 2 scenarios (2D-200obs, 3D-500obs)")
    print("  - 10 repeats per group")
    print("  - Total runs: 12 x 2 x 10 = 240")
    print("  - Estimated time: 15-18 minutes")
    
    # 创建实验对象
    experiment = PIDExperiment(
        quick_test=True,  # 使用快速模式的重复次数
        fine_tuning=True,  # 启用三参数精细调优
        output_dir='results'
    )
    
    # 运行实验
    print("\nStarting fine-tuning experiment...")
    results_df = experiment.run_experiment()
    
    return results_df, experiment


def run_quick_test():
    """运行大批量实验（完整统计分析版）"""
    print_header("LARGE-SCALE EXPERIMENT - High Success Rate Config")
    print("\nExperiment Configuration:")
    print("  - 10 PID parameter sets (1 Fixed + 2 Low + 7 High-Gain Dense)")
    print("  - 2 scenarios (2D-150obs, 3D-250obs)")
    print("  - 30 repeats per group [HIGH STATISTICAL CONFIDENCE]")
    print("  - 2D: 750 iterations, 3D: 600 iterations [60-90% success rate]")
    print("  - Total runs: 10 x 2 x 30 = 600")
    print("  - Estimated time: 35-45 minutes")
    
    # 创建实验对象
    experiment = PIDExperiment(
        quick_test=True,  # 使用优化的迭代配置
        output_dir='results'
    )
    
    # 运行实验
    print("\nStarting large-scale experiment...")
    results_df = experiment.run_experiment()
    
    return results_df, experiment


def run_full_experiment():
    """运行完整实验（完整统计分析版）"""
    print_header("FULL EXPERIMENT MODE - Complete Analysis")
    print("\nExperiment Configuration:")
    print("  - 9 PID parameter sets (Kp: 0.05~0.8)")
    print("  - 2 scenarios (2D-200obs, 3D-500obs)")
    print("  - 30 repeats per group [MAXIMUM STATISTICAL CONFIDENCE]")
    print("  - 2D: 600 iterations, 3D: 950 iterations [75%+ success rate]")
    print("  - Total runs: 9 x 2 x 30 = 540")
    print("  - Estimated time: 45-60 minutes")
    
    # 创建实验对象
    experiment = PIDExperiment(
        quick_test=False,  # 完整实验模式
        output_dir='results'
    )
    
    # 运行实验
    print("\nStarting full experiment...")
    results_df = experiment.run_experiment()
    
    return results_df, experiment


def analyze_and_visualize(results_df, experiment):
    """分析结果并生成可视化"""
    print_header("结果分析与可视化")
    
    # 保存结果
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results_file = experiment.output_dir / f'pid_results_{timestamp}.csv'
    results_df.to_csv(results_file, index=False, encoding='utf-8-sig')
    print(f"\n[SAVED] Results: {results_file}")
    
    # 创建分析器
    analyzer = ResultAnalyzer(str(results_file))
    
    # 生成统计摘要（三层评价体系）
    print("\n[ANALYZING] Generating statistical summary...")
    summary_df = analyzer.generate_summary_statistics()
    
    # 保存摘要
    summary_file = experiment.output_dir / f'analysis_summary_{timestamp}.csv'
    summary_df.to_csv(summary_file, index=False, encoding='utf-8-sig')
    print(f"[SAVED] Summary: {summary_file}")
    
    # ★ 分层最优判定 ★
    print("\n[OPTIMAL] Finding optimal PID configuration...")
    try:
        optimal_pid = analyzer.find_optimal_pid(success_threshold=0.70)
    except Exception as e:
        print(f"[WARNING] Optimal PID search failed: {e}")
        optimal_pid = None
    
    # 生成可视化图表
    print("\n[PLOTTING] Generating visualization plots...")
    try:
        analyzer.generate_all_plots()
        print(f"[SAVED] Generated visualization plots")
    except Exception as e:
        print(f"[WARNING] Visualization failed: {e}")
        import traceback
        traceback.print_exc()
    
    return summary_file, optimal_pid


def main():
    """主函数"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='SC-RRT PID有效性验证实验 - 采样行为分析')
    parser.add_argument('--quick', action='store_true', help='快速实验 (240 runs, ~15-18分钟)')
    parser.add_argument('--refine', action='store_true', help='精细化参数搜索 (280 runs, ~20-25分钟)')
    parser.add_argument('--finetune', action='store_true', help='三参数调优 (240 runs, ~15-18分钟)')
    parser.add_argument('--baseline', action='store_true', help='【推荐】PID有效性验证实验 (240 runs, ~15-18分钟)')
    args = parser.parse_args()
    
    try:
        # 运行实验（默认使用baseline模式）
        if args.baseline or (not args.quick and not args.refine and not args.finetune):
            results_df, experiment = run_baseline_experiment()
        elif args.finetune:
            results_df, experiment = run_fine_tuning()
        elif args.refine:
            results_df, experiment = run_refined_search()
        elif args.quick:
            results_df, experiment = run_quick_test()
        else:
            results_df, experiment = run_full_experiment()
        
        # 分析结果
        summary_file, optimal_pid = analyze_and_visualize(results_df, experiment)
        
        print_header("EXPERIMENT COMPLETED!")
        print(f"\nAll results saved to: {experiment.output_dir}")
        print("\nGenerated files:")
        print("  - pid_results_*.csv           : Raw data")
        print("  - summary_statistics.csv      : Complete statistics")
        print("  - optimal_pid_result.txt      : Optimal PID recommendation")
        print("  - statistical_summary_*.txt   : Statistics with ANOVA")
        print("  - anova_analysis_*.txt        : ANOVA analysis")
        print("  - latex_table_*.tex           : LaTeX table for paper")
        print("  - *.png                       : High-quality figures (300dpi)")
        
        if optimal_pid is not None:
            print(f"\n🎯 推荐的最优PID配置: {optimal_pid['pid_config']}")
            print(f"   参数: Kp={optimal_pid['Kp']:.2f}, Ki={optimal_pid['Ki']:.2f}, Kd={optimal_pid['Kd']:.2f}")
        
    except KeyboardInterrupt:
        print("\n\nExperiment interrupted by user")
        sys.exit(0)
    except Exception as e:
        print(f"\n\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
