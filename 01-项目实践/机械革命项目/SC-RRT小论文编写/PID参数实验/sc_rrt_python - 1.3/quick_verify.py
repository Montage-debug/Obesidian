"""
快速验证迭代次数调整后的成功率
只测试3个关键配置，验证是否达到60%+成功率
"""
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent))

from experiments.run_pid_experiment import PIDExperiment

class QuickVerify:
    """快速验证实验"""
    
    def __init__(self):
        # 只测试3个关键配置
        self.test_configs = [
            {'Kp': 0.20, 'Ki': 0.02, 'Kd': 0.08, 'name': 'OPTIMAL'},      # 当前最优
            {'Kp': 0.20, 'Ki': 0.04, 'Kd': 0.08, 'name': 'Alternative1'}, # 次优
            {'Kp': 0.20, 'Ki': 0.01, 'Kd': 0.05, 'name': 'Alternative2'}  # 参考
        ]
    
    def run(self):
        """运行快速验证"""
        print("\n" + "="*70)
        print("快速验证：迭代次数调整后成功率测试")
        print("="*70)
        print("\n目标：验证成功率是否达到60%+")
        print("测试配置：3个关键PID配置")
        print("每配置：2场景 × 10重复 = 20次")
        print("总运行：60次，预计3-4分钟\n")
        
        # 创建实验对象（使用快速模式，会自动使用新的迭代次数）
        experiment = PIDExperiment(
            quick_test=True,
            output_dir='results/verify'
        )
        
        # 手动替换PID配置为验证配置
        experiment.pid_configs = self.test_configs
        
        print(f"实验配置:")
        print(f"  二维迭代: {experiment.max_iterations_2d}")
        print(f"  三维迭代: {experiment.max_iterations_3d}")
        print(f"  场景数: {len(experiment.scenarios)}")
        print(f"  参数组: {len(self.test_configs)}")
        print(f"  重复次数: {experiment.num_trials}")
        print(f"  总实验数: {len(experiment.scenarios) * len(self.test_configs) * experiment.num_trials}\n")
        
        # 运行实验
        results_df = experiment.run_experiment()
        
        # 快速统计
        print("\n" + "="*70)
        print("验证结果统计")
        print("="*70)
        
        for config in self.test_configs:
            config_name = config['name']
            config_data = results_df[results_df['pid_config'] == config_name]
            
            total = len(config_data)
            success = config_data['success'].sum()
            success_rate = (success / total * 100) if total > 0 else 0
            
            status = "✓ 通过" if success_rate >= 60 else "✗ 未达标"
            print(f"\n{config_name} (Kp={config['Kp']}, Ki={config['Ki']}, Kd={config['Kd']})")
            print(f"  成功率: {success_rate:.1f}% ({success}/{total}) {status}")
        
        # 总体统计
        total_success = results_df['success'].sum()
        total_runs = len(results_df)
        overall_rate = (total_success / total_runs * 100) if total_runs > 0 else 0
        
        print(f"\n{'='*70}")
        print(f"总体成功率: {overall_rate:.1f}% ({total_success}/{total_runs})")
        
        if overall_rate >= 60:
            print("✓✓✓ 验证通过！可以进行大批量实验")
        else:
            print(f"✗✗✗ 未达标，建议进一步增加迭代次数")
        
        print("="*70 + "\n")
        
        return results_df, overall_rate

if __name__ == '__main__':
    verifier = QuickVerify()
    results, success_rate = verifier.run()
