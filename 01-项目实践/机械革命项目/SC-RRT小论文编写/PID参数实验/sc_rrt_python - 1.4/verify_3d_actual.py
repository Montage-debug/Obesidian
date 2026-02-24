"""
验证三维空间实际使用的配置
"""
import sys
import os

# 添加项目根目录到Python路径
project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

import numpy as np
from src.environment import EnvironmentConfig

# 生成三维环境（使用实验中的配置）
env = EnvironmentConfig.generate_3d_environment(
    bounds=[0, 1500, 0, 1500, 0, 1500],
    num_obstacles=400,
    start_point=np.array([75, 75, 75]),
    goal_point=np.array([1425, 1425, 1425]),
    radius_range=(45, 70),
    min_spacing=18,
    clearance=22,
    seed=302
)

obstacles = env['obstacles']
num_placed = len(obstacles)

print("="*70)
print("【三维空间实际配置验证】")
print("="*70)

print(f"\n目标障碍物数量: 400")
print(f"实际放置数量: {num_placed}")
print(f"放置率: {num_placed/400*100:.1f}%")

if num_placed > 0:
    radii = obstacles[:, 3]
    print(f"\n障碍物半径范围: [{radii.min():.1f}, {radii.max():.1f}]")
    print(f"障碍物平均半径: {radii.mean():.1f}")
    print(f"障碍物半径标准差: {radii.std():.1f}")
    
    # 计算空间占用率
    total_volume = 1500 ** 3
    obstacle_volume = np.sum(4/3 * np.pi * radii**3)
    occupancy = obstacle_volume / total_volume * 100
    
    print(f"\n总空间: {total_volume:.0f}")
    print(f"障碍物占用: {obstacle_volume:.0f}")
    print(f"空间占用率: {occupancy:.2f}%")
    
    print(f"\n配置参数 (代码中的):")
    print(f"  radius_range: (45, 70)")
    print(f"  min_spacing: 18")
    print(f"  clearance: 22")
    
    # 评估难度
    print(f"\n【难度评估】")
    if occupancy < 8:
        print(f"❌ 太简单：占用率{occupancy:.2f}% < 8%")
        print("   预期成功率：No_PID > 70%, Optimal_PID > 85%")
    elif occupancy < 12:
        print(f"✓ 适中：占用率{occupancy:.2f}% 在8-12%之间")
        print("   预期成功率：No_PID 40-50%, Optimal_PID 65-75%")
    else:
        print(f"⚠ 较难：占用率{occupancy:.2f}% > 12%")
        print("   预期成功率：No_PID < 35%, Optimal_PID 50-60%")
    
    # 分析实验结果
    print(f"\n【实验结果分析】")
    print("观察到的现象：三维空间大量成功")
    print(f"实际配置：R{radii.min():.0f}-{radii.max():.0f}, 占用率{occupancy:.2f}%")
    
    if occupancy < 10:
        print("\n【结论】")
        print("虽然代码中使用了R45-70配置，但9.49%的占用率仍然偏低")
        print("建议：进一步减小半径范围或增加障碍物数量")
        print("\n推荐配置1: radius_range=(35, 55), 目标占用率~10-12%")
        print("推荐配置2: radius_range=(40, 60), num_obstacles=500, 目标占用率~11-13%")

print("\n" + "="*70)
