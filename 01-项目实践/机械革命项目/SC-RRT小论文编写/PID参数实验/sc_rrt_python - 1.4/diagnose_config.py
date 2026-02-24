"""
诊断脚本：检查实验运行时实际使用的障碍物配置
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent / 'src'))

import numpy as np
from experiments.run_pid_experiment import PIDExperiment

def diagnose_experiment_config():
    """诊断实验配置"""
    print("\n" + "="*70)
    print("实验配置诊断工具")
    print("="*70)
    
    # 创建实验对象
    experiment = PIDExperiment(
        quick_test=True,
        baseline_mode=True,
        output_dir='results'
    )
    
    print("\n【实验对象配置】")
    print(f"场景数量: {len(experiment.scenarios)}")
    print(f"PID配置数: {len(experiment.pid_configs)}")
    print(f"重复次数: {experiment.num_trials}")
    print(f"二维迭代: {experiment.max_iterations_2d}")
    print(f"三维迭代: {experiment.max_iterations_3d}")
    
    # 检查每个场景的障碍物配置
    for i, scenario in enumerate(experiment.scenarios, 1):
        print(f"\n{'='*70}")
        print(f"场景{i}: {scenario['name']}")
        print(f"{'='*70}")
        
        env = scenario['env']
        dim = scenario['dimension']
        obstacles = env['obstacles']
        
        print(f"维度: {dim}D")
        print(f"空间边界: {env['bounds']}")
        print(f"起点: {env['start']}")
        print(f"终点: {env['goal']}")
        print(f"步长: {scenario['step_size']}")
        print(f"目标阈值: {scenario['goal_threshold']}")
        print(f"最大迭代: {scenario['max_iterations']}")
        
        print(f"\n【障碍物信息】")
        print(f"目标数量: 未知（需查看生成代码）")
        print(f"实际数量: {len(obstacles)}")
        
        if len(obstacles) > 0:
            if dim == 2:
                radii = obstacles[:, 2]
            else:
                radii = obstacles[:, 3]
            
            print(f"半径统计:")
            print(f"  最小值: {np.min(radii):.2f}")
            print(f"  最大值: {np.max(radii):.2f}")
            print(f"  平均值: {np.mean(radii):.2f}")
            print(f"  中位数: {np.median(radii):.2f}")
            print(f"  标准差: {np.std(radii):.2f}")
            
            # 计算空间占用率
            if dim == 2:
                total_volume = np.sum(np.pi * radii**2)
                bounds = env['bounds']
                space_volume = (bounds[1] - bounds[0]) * (bounds[3] - bounds[2])
                unit = "平方单位"
            else:
                total_volume = np.sum((4/3) * np.pi * radii**3)
                bounds = env['bounds']
                space_volume = (bounds[1] - bounds[0]) * (bounds[3] - bounds[2]) * (bounds[5] - bounds[4])
                unit = "立方单位"
            
            occupancy_rate = total_volume / space_volume * 100
            print(f"\n空间占用率: {occupancy_rate:.4f}%")
            print(f"总体积: {total_volume:,.0f} {unit}")
            print(f"空间体积: {space_volume:,.0f} {unit}")
            
            # 难度评估
            if dim == 2:
                if occupancy_rate < 20:
                    difficulty = "低 ⭐⭐"
                elif occupancy_rate < 30:
                    difficulty = "中 ⭐⭐⭐"
                elif occupancy_rate < 40:
                    difficulty = "高 ⭐⭐⭐⭐"
                else:
                    difficulty = "极高 ⭐⭐⭐⭐⭐"
            else:
                if occupancy_rate < 5:
                    difficulty = "极低 ⭐"
                elif occupancy_rate < 8:
                    difficulty = "低 ⭐⭐"
                elif occupancy_rate < 12:
                    difficulty = "中 ⭐⭐⭐"
                elif occupancy_rate < 16:
                    difficulty = "高 ⭐⭐⭐⭐"
                else:
                    difficulty = "极高 ⭐⭐⭐⭐⭐"
            
            print(f"难度评级: {difficulty}")
            
            # 检查直线路径是否被阻挡
            start = env['start']
            goal = env['goal']
            line_vec = goal - start
            line_len = np.linalg.norm(line_vec)
            line_dir = line_vec / line_len
            
            blocking_count = 0
            for obs in obstacles:
                if dim == 2:
                    obs_center = obs[:2]
                    obs_radius = obs[2]
                else:
                    obs_center = obs[:3]
                    obs_radius = obs[3]
                
                point_vec = obs_center - start
                proj_len = np.dot(point_vec, line_dir)
                proj_len = np.clip(proj_len, 0, line_len)
                
                closest_point = start + proj_len * line_dir
                dist_to_line = np.linalg.norm(obs_center - closest_point)
                
                if dist_to_line < obs_radius * 1.1:
                    blocking_count += 1
            
            print(f"\n直线路径阻挡:")
            print(f"  阻挡障碍物数: {blocking_count}/{len(obstacles)}")
            print(f"  状态: {'被阻挡 ✗' if blocking_count > 0 else '畅通 ✓'}")
    
    # 检查PID配置
    print(f"\n{'='*70}")
    print("PID配置列表")
    print(f"{'='*70}")
    for i, cfg in enumerate(experiment.pid_configs, 1):
        mode = cfg.get('mode', 'custom_pid')
        print(f"\n配置{i}: {cfg['name']}")
        print(f"  Kp={cfg['Kp']:.2f}, Ki={cfg['Ki']:.2f}, Kd={cfg['Kd']:.2f}")
        print(f"  模式: {mode}")
    
    print(f"\n{'='*70}")
    print("诊断完成")
    print(f"{'='*70}")


if __name__ == '__main__':
    diagnose_experiment_config()
