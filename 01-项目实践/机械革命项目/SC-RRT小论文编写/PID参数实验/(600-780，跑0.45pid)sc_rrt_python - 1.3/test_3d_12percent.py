"""
测试三维空间达到12%占用率的可行方案
"""
import sys
import os

project_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, project_root)

import numpy as np
from src.environment import EnvironmentConfig

print("="*70)
print("【三维空间12%占用率方案测试】")
print("="*70)

configs = [
    # (radius_min, radius_max, num_obstacles, min_spacing, clearance, 描述)
    (38, 62, 500, 16, 20, "当前配置"),
    (46, 70, 500, 16, 20, "方案2: N500, R46-70"),
    (42, 66, 600, 15, 19, "方案3: N600, R42-66"),
    (38, 62, 750, 14, 18, "方案1: N750, R38-62"),
    (45, 68, 550, 15, 19, "折中: N550, R45-68"),
    (48, 72, 525, 15, 19, "激进: N525, R48-72"),
]

print(f"\n{'配置':<30} {'放置':<12} {'占用率':<10} {'评分':<8} 状态")
print("-" * 75)

best = None
best_score = -999

for r_min, r_max, num_obs, spacing, clear, desc in configs:
    env = EnvironmentConfig.generate_3d_environment(
        bounds=[0, 1500, 0, 1500, 0, 1500],
        num_obstacles=num_obs,
        start_point=np.array([75, 75, 75]),
        goal_point=np.array([1425, 1425, 1425]),
        radius_range=(r_min, r_max),
        min_spacing=spacing,
        clearance=clear,
        seed=302
    )
    
    num_placed = len(env['obstacles'])
    placement_rate = num_placed / num_obs * 100
    
    if num_placed > 0:
        radii = env['obstacles'][:, 3]
        total_volume = 1500 ** 3
        obstacle_volume = np.sum(4/3 * np.pi * radii**3)
        occupancy = obstacle_volume / total_volume * 100
    else:
        occupancy = 0
    
    # 评分：100%放置 + 11-13%占用
    score = 0
    
    if placement_rate >= 99:
        score += 100
    elif placement_rate >= 95:
        score += 80
    else:
        score += placement_rate * 0.6
    
    if 11 <= occupancy <= 13:
        score += 100
    elif 10 <= occupancy < 11:
        score += 80 + (occupancy - 10) * 20
    elif 13 < occupancy <= 14:
        score += 90
    else:
        score += max(0, 100 - abs(occupancy - 12) * 8)
    
    status = ""
    if placement_rate >= 99 and 11 <= occupancy <= 13:
        status = "✓✓ 完美"
    elif placement_rate >= 99 and 10 <= occupancy <= 14:
        status = "✓ 优秀"
    elif placement_rate >= 95 and 11 <= occupancy <= 13:
        status = "○ 良好"
    else:
        status = "△ 一般"
    
    config_str = f"{desc} (R{r_min}-{r_max} N{num_obs})"
    placed_str = f"{num_placed}/{num_obs}"
    print(f"{config_str:<30} {placed_str:<12} {occupancy:>7.2f}%   {score:>6.1f}   {status}")
    
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
    print("【最佳方案】")
    print(f"  方案: {best['desc']}")
    print(f"  radius_range: {best['radius_range']}")
    print(f"  num_obstacles: {best['num_obstacles']}")
    print(f"  min_spacing: {best['min_spacing']}")
    print(f"  clearance: {best['clearance']}")
    print(f"  实际放置: {best['placed']}/{best['num_obstacles']} ({best['placement_rate']:.1f}%)")
    print(f"  占用率: {best['occupancy']:.2f}%")
    print(f"  评分: {best['score']:.1f}/200")
    
    if best['occupancy'] >= 11:
        print("\n✓ 达到目标占用率（11-13%）")
    else:
        print(f"\n⚠ 占用率{best['occupancy']:.2f}%仍低于目标")

print("="*70)
