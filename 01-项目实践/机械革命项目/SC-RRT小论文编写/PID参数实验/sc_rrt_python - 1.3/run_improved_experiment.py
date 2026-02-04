"""
SC-RRT PID Parameter Optimization Experiment - IMPROVED VERSION
改进版PID参数优化实验

关键改进:
1. ✅ 提高迭代次数 (2D: 900, 3D: 1200)
2. ✅ 优化PID参数搜索范围
3. ✅ 增加No_PID和Fixed_Ellipsoid对照组
4. ✅ 调整障碍物密度到合理范围
5. ✅ 增加ESR目标优化
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


def run_improved_experiment():
    """运行改进版PID有效性验证实验"""
    print_header("IMPROVED PID EXPERIMENT - V8.0")
    print("\n" + "="*70)
    print("【关键改进】")
    print("="*70)
    print("\n✅ 1. 迭代次数优化")
    print("    • 2D场景: 900次 (原600次，提升50%)")
    print("    • 3D场景: 1200次 (原750次，提升60%)")
    print("    • 目标成功率: 70-85% (原30-40%)")
    print("\n✅ 2. PID参数范围优化")
    print("    • 新增超低档: Kp=0.05 (探索边界)")
    print("    • 优化低档: Kp=0.08 (基线)")
    print("    • 扩展中档: Kp=0.10-0.18 (精细搜索)")
    print("    • 保留对照: No_PID (Kp=0.0)")
    print("\n✅ 3. 障碍物密度调整")
    print("    • 2D: 200个障碍物 (原213，降低难度)")
    print("    • 3D: 350个障碍物 (原320，适度增加)")
    print("\n✅ 4. 对照组完善")
    print("    • No_PID: 完全无约束 (ESR=0%)")
    print("    • Low_PID: Kp=0.05-0.10 (弱约束)")
    print("    • Optimal_PID: Kp=0.12-0.18 (目标区间)")
    print("    • High_PID: Kp=0.20+ (强约束)")
    print("\n【实验配置】")
    print("  配置数: 9 组 (增加3个配置)")
    print("  场景数: 2 个")
    print("  重复次数: 10 次/配置")
    print("  总运行数: 9 × 2 × 10 = 180 次")
    print("  预计耗时: 25-35分钟")
    print("\n【预期结果】")
    print("  成功率: No_PID=60-70%, Optimal_PID=75-85%")
    print("  ESR: No_PID=0%, Optimal_PID=50-65%")
    print("  Sample-to-Success: Optimal_PID比No_PID减少40-50%")
    print("="*70)
    
    # 创建实验对象
    experiment = PIDExperiment(
        quick_test=True,
        baseline_mode=False,  # 使用改进模式
        output_dir='results_improved'
    )
    
    # 覆盖默认配置
    experiment._override_configs_improved()
    
    # 运行实验
    print("\n开始改进版PID有效性验证实验...")
    results_df = experiment.run_experiment()
    
    return results_df, experiment


def run_quick_test():
    """快速测试（5分钟验证）"""
    print_header("QUICK TEST MODE - 5 Minutes")
    print("\nTest Configuration:")
    print("  - 3 PID configs (No_PID, Low, Optimal)")
    print("  - 1 scenario (2D only)")
    print("  - 5 repeats")
    print("  - Total: 15 runs (~5 minutes)")
    
    experiment = PIDExperiment(
        quick_test=True,
        baseline_mode=False,
        output_dir='results_quick_test'
    )
    
    # 设置快速测试配置
    experiment.num_trials = 5
    experiment.scenarios = [experiment.scenarios[0]]  # 只用2D场景
    experiment.pid_configs = experiment.pid_configs[:3]  # 只用前3个配置
    
    print("\n运行快速测试...")
    results_df = experiment.run_experiment()
    
    return results_df, experiment


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='SC-RRT PID参数优化实验 - 改进版')
    parser.add_argument('--quick', action='store_true', 
                       help='快速测试模式 (5分钟)')
    parser.add_argument('--mode', type=str, default='improved',
                       choices=['improved', 'quick'],
                       help='实验模式: improved(完整改进), quick(快速测试)')
    
    args = parser.parse_args()
    
    start_time = datetime.now()
    
    try:
        if args.quick or args.mode == 'quick':
            results_df, experiment = run_quick_test()
        else:
            results_df, experiment = run_improved_experiment()
        
        # 分析结果
        print_header("结果分析与可视化")
        analyzer = ResultAnalyzer(experiment.output_dir)
        analyzer.generate_full_report(results_df)
        
        # 计算总耗时
        elapsed = (datetime.now() - start_time).total_seconds() / 60
        
        print_header("EXPERIMENT COMPLETED!")
        print(f"\n所有结果已保存到: {experiment.output_dir}")
        print(f"总耗时: {elapsed:.1f}分钟")
        print("\n生成的文件:")
        print("  - pid_results_*.csv           : 原始数据")
        print("  - summary_statistics.csv      : 汇总统计")
        print("  - optimal_pid_result.txt      : 最优PID推荐")
        print("  - *.png                       : 可视化图表")
        
    except KeyboardInterrupt:
        print("\n\n实验被用户中断")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
