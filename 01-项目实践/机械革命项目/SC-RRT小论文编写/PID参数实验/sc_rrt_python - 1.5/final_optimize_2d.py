"""
二维配置最终优化
策略：保持数量不变，通过调整半径和间距达到目标
"""
import sys
import os

project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

import numpy as np
from src.environment import EnvironmentConfig

print("="*70)
print("【二维空间最终优化方案】")
print("="*70)

print("\n【策略】")
print("  保持数量: 200-225个障碍物（不要太少）")
print("  目标放置率: ≥99%")
print("  目标占用率: 25-30%（保持高难度）")
print("  方法: 缩小半径范围，优化间距")

# 精细测试
configs = [
    # (radius_min, radius_max, num_obstacles, min_spacing, clearance)
    (22, 40, 200, 15, 18),
    (23, 41, 200, 15, 18),
    (24, 42, 200, 15, 18),
    (22, 40, 210, 14, 17),
    (23, 41, 210, 14, 17),
    (24, 42, 210, 14, 17),
    (22, 40, 215, 14, 17),
    (23, 41, 215, 14, 17),
    (21, 39, 215, 15, 18),
    (22, 40, 220, 13, 16),
]

print(f"\n{'配置':<28} {'放置':<12} {'占用率':<10} {'评分':<8} 状态")
print("-" * 75)

best = None
best_score = -999

for r_min, r_max, num_obs, spacing, clear in configs:
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
    
    # 评分
    score = 0
    
    # 放置率评分（必须≥99%）
    if placement_rate >= 99:
        score += 100
    elif placement_rate >= 97:
        score += 85
    elif placement_rate >= 95:
        score += 70
    else:
        score += placement_rate * 0.5
    
    # 占用率评分（25-30%最佳）
    if 25 <= occupancy <= 30:
        score += 100
    elif 24 <= occupancy < 25:
        score += 90 + (occupancy - 24) * 10
    elif 30 < occupancy <= 31:
        score += 90 + (31 - occupancy) * 10
    elif 23 <= occupancy < 24:
        score += 80 + (occupancy - 23) * 10
    elif 22 <= occupancy < 23:
        score += 70 + (occupancy - 22) * 10
    else:
        score += max(0, 100 - abs(occupancy - 27.5) * 5)
    
    status = ""
    if placement_rate >= 99 and 25 <= occupancy <= 30:
        status = "✓✓ 完美"
    elif placement_rate >= 99 and 24 <= occupancy < 25:
        status = "✓ 优秀"
    elif placement_rate >= 99 and 22 <= occupancy < 24:
        status = "○ 良好"
    elif placement_rate >= 97 and 25 <= occupancy <= 30:
        status = "△ 可用"
    else:
        status = "✗ 不佳"
    
    config_str = f"R{r_min}-{r_max} N{num_obs} S{spacing}"
    placed_str = f"{num_placed}/{num_obs}"
    print(f"{config_str:<28} {placed_str:<12} {occupancy:>7.2f}%   {score:>6.1f}   {status}")
    
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
            'score': score
        }

print("\n" + "="*70)
if best:
    print("【推荐配置】")
    print(f"  radius_range: {best['radius_range']}")
    print(f"  num_obstacles: {best['num_obstacles']}")
    print(f"  min_spacing: {best['min_spacing']}")
    print(f"  clearance: {best['clearance']}")
    print(f"  实际放置: {best['placed']}/{best['num_obstacles']} ({best['placement_rate']:.1f}%)")
    print(f"  占用率: {best['occupancy']:.2f}%")
    print(f"  评分: {best['score']:.1f}/200")
    
    print("\n【总结】")
    print(f"  ✓ 保持了障碍物数量（{best['num_obstacles']}个）")
    print(f"  ✓ 放置率: {best['placement_rate']:.1f}%")
    
    if best['occupancy'] >= 25:
        print(f"  ✓ 占用率{best['occupancy']:.2f}%在目标范围内（25-30%）")
    elif best['occupancy'] >= 22:
        print(f"  ○ 占用率{best['occupancy']:.2f}%略低于目标，但可接受")
    else:
        print(f"  ⚠ 占用率{best['occupancy']:.2f}%偏低，难度不足")

print("="*70)
