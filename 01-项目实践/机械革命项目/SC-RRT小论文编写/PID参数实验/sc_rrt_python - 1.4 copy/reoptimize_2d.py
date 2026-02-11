"""
重新优化二维配置
目标：100%放置率 + 25-30%占用率（保持原有难度）
"""
import sys
import os

project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

import numpy as np
from src.environment import EnvironmentConfig

print("="*70)
print("【二维空间配置重新优化】")
print("="*70)

print("\n【问题分析】")
print("  原配置: R25-45, N225")
print("    → 只能放置178个(79.1%), 占用率30.42%")
print("    → 问题: 放不下目标数量，而非占用率过高")
print("\n  错误推荐: R20-35, N225")
print("    → 能放置225个(100%), 但占用率降到23.41%")
print("    → 问题: 难度降低了，不是我们想要的")

print("\n【正确目标】")
print("  放置率: 100% (能放下所有障碍物)")
print("  占用率: 25-30% (保持原有难度)")

# 测试配置：减少数量而不是减小半径
configs = [
    # (radius_min, radius_max, num_obstacles, min_spacing, clearance, 描述)
    (25, 45, 225, 15, 18, "原配置"),
    (25, 45, 200, 16, 20, "减少数量到200"),
    (25, 45, 180, 16, 20, "减少数量到180"),
    (22, 42, 200, 16, 20, "略减半径+数量200"),
    (23, 43, 200, 16, 20, "微调半径+数量200"),
    (24, 44, 195, 16, 20, "微调半径+数量195"),
    (25, 45, 190, 16, 20, "原半径+数量190"),
    (26, 46, 190, 16, 20, "略增半径+数量190"),
]

print(f"\n{'配置':<25} {'放置':<12} {'占用率':<10} {'评分':<8} 状态")
print("-" * 75)

best = None
best_score = -999

for r_min, r_max, num_obs, spacing, clear, desc in configs:
    env = EnvironmentConfig.generate_2d_environment(
        bounds=[0, 1500, 0, 1500],
        num_obstacles=num_obs,
        start_point=np.array([75, 75]),
        goal_point=np.array([1425, 1425]),
        radius_range=(r_min, r_max),
        min_spacing=spacing,
        clearance=clear,
        seed=301
    )
    
    num_placed = len(env['obstacles'])
    placement_rate = num_placed / num_obs * 100
    
    if num_placed > 0:
        radii = env['obstacles'][:, 2]
        total_area = 1500 * 1500
        obstacle_area = np.sum(np.pi * radii**2)
        occupancy = obstacle_area / total_area * 100
    else:
        occupancy = 0
    
    # 评分：目标是100%放置率 + 25-30%占用率
    score = 0
    
    # 放置率评分
    if placement_rate >= 99:
        score += 100
    elif placement_rate >= 95:
        score += 80
    else:
        score += placement_rate * 0.6
    
    # 占用率评分（目标25-30%）
    if 25 <= occupancy <= 30:
        score += 100
    elif 23 <= occupancy < 25:
        score += 80 + (occupancy - 23) * 10
    elif 30 < occupancy <= 32:
        score += 80 + (32 - occupancy) * 10
    elif 20 <= occupancy < 23:
        score += 60 + (occupancy - 20) * 6.67
    else:
        score += max(0, 100 - abs(occupancy - 27.5) * 5)
    
    status = ""
    if placement_rate >= 99 and 25 <= occupancy <= 30:
        status = "✓ 优秀"
    elif placement_rate >= 99 and 23 <= occupancy < 25:
        status = "○ 接近"
    elif placement_rate >= 95 and 25 <= occupancy <= 30:
        status = "△ 可用"
    else:
        status = "✗ 不佳"
    
    config_str = f"R{r_min}-{r_max} N{num_obs}"
    placed_str = f"{num_placed}/{num_obs}"
    print(f"{config_str:<25} {placed_str:<12} {occupancy:>7.2f}%   {score:>6.1f}   {status}")
    
    if score > best_score:
        best_score = score
        best = {
            'radius_range': (r_min, r_max),
            'num_obstacles': num_obs,
            'min_spacing': spacing,
            'clearance': clear,
            'placement_rate': placement_rate,
            'placed': num_placed,
            'occupancy': occupancy,
            'score': score,
            'desc': desc
        }

print("\n" + "="*70)
if best:
    print("【最佳配置】")
    print(f"  描述: {best['desc']}")
    print(f"  radius_range: {best['radius_range']}")
    print(f"  num_obstacles: {best['num_obstacles']}")
    print(f"  min_spacing: {best['min_spacing']}")
    print(f"  clearance: {best['clearance']}")
    print(f"  实际放置: {best['placed']}/{best['num_obstacles']} ({best['placement_rate']:.1f}%)")
    print(f"  占用率: {best['occupancy']:.2f}%")
    print(f"  评分: {best['score']:.1f}/200")
    
    print("\n【对比原配置】")
    print(f"  原配置: R25-45, N225 → 178个(79.1%), 30.42%占用率")
    print(f"  新配置: R{best['radius_range'][0]}-{best['radius_range'][1]}, N{best['num_obstacles']} → {best['placed']}个({best['placement_rate']:.1f}%), {best['occupancy']:.2f}%占用率")
    
    if best['occupancy'] >= 25:
        print("\n✓ 成功：保持了高难度（25-30%），且能100%放置")
    elif best['occupancy'] >= 23:
        print("\n○ 接近：占用率略低于目标，但可接受")
    else:
        print("\n✗ 难度降低：占用率<23%，可能太简单")

print("="*70)
