"""
快速验证三维配置 - 测试障碍物放置率和路径规划成功率
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent / 'src'))

import numpy as np
from src.environment import EnvironmentConfig

def test_configuration(config_name, **kwargs):
    """测试配置的可行性"""
    print(f"\n{'='*70}")
    print(f"测试配置: {config_name}")
    print(f"{'='*70}")
    
    env = EnvironmentConfig.generate_3d_environment(**kwargs)
    
    actual = len(env['obstacles'])
    target = kwargs['num_obstacles']
    placement_rate = actual / target * 100
    
    # 计算体积占比
    total_volume = 0
    for obs in env['obstacles']:
        radius = obs[3]
        total_volume += (4/3) * np.pi * radius**3
    
    bounds = kwargs['bounds']
    space_volume = (bounds[1] - bounds[0]) * (bounds[3] - bounds[2]) * (bounds[5] - bounds[4])
    occupancy_rate = total_volume / space_volume * 100
    
    avg_radius = np.mean(env['obstacles'][:, 3]) if len(env['obstacles']) > 0 else 0
    
    print(f"\n【放置结果】")
    print(f"  目标数量: {target}")
    print(f"  实际放置: {actual}")
    print(f"  放置率: {placement_rate:.1f}%")
    
    print(f"\n【空间占用】")
    print(f"  平均半径: {avg_radius:.2f}")
    print(f"  体积占比: {occupancy_rate:.4f}%")
    
    print(f"\n【难度评估】")
    if placement_rate >= 95:
        difficulty = "偏低 ⭐⭐"
        recommendation = "可以适当增大半径"
    elif placement_rate >= 85:
        difficulty = "适中 ⭐⭐⭐"
        recommendation = "✓ 合适的配置"
    elif placement_rate >= 70:
        difficulty = "较高 ⭐⭐⭐⭐"
        recommendation = "✓ 良好的挑战性"
    elif placement_rate >= 50:
        difficulty = "很高 ⭐⭐⭐⭐⭐"
        recommendation = "建议适当减小半径"
    else:
        difficulty = "极高 ⭐⭐⭐⭐⭐"
        recommendation = "⚠ 半径过大，建议降低"
    
    print(f"  难度等级: {difficulty}")
    print(f"  建议: {recommendation}")
    
    return placement_rate, occupancy_rate


if __name__ == '__main__':
    print("\n" + "="*70)
    print("三维配置快速验证工具")
    print("="*70)
    
    # 测试当前配置 (R55-85)
    rate1, occ1 = test_configuration(
        "当前配置 (R55-85)",
        bounds=[0, 1500, 0, 1500, 0, 1500],
        num_obstacles=400,
        start_point=np.array([75, 75, 75]),
        goal_point=np.array([1425, 1425, 1425]),
        radius_range=(55, 85),
        min_spacing=22,
        clearance=25,
        seed=302
    )
    
    # 测试更保守的配置 (R50-80)
    rate2, occ2 = test_configuration(
        "保守配置 (R50-80)",
        bounds=[0, 1500, 0, 1500, 0, 1500],
        num_obstacles=400,
        start_point=np.array([75, 75, 75]),
        goal_point=np.array([1425, 1425, 1425]),
        radius_range=(50, 80),
        min_spacing=20,
        clearance=23,
        seed=302
    )
    
    # 测试更激进的配置 (R60-90)
    rate3, occ3 = test_configuration(
        "激进配置 (R60-90)",
        bounds=[0, 1500, 0, 1500, 0, 1500],
        num_obstacles=400,
        start_point=np.array([75, 75, 75]),
        goal_point=np.array([1425, 1425, 1425]),
        radius_range=(60, 90),
        min_spacing=23,
        clearance=27,
        seed=302
    )
    
    # 总结
    print(f"\n{'='*70}")
    print("配置对比总结")
    print(f"{'='*70}")
    print(f"\n{'配置':<20} {'放置率':<12} {'占用率':<12} {'推荐'}")
    print("-"*70)
    print(f"{'R55-85(当前)':<20} {rate1:>6.1f}% {'':<6} {occ1:>6.4f}% {'':<6} {'✓ 适中' if 85 <= rate1 <= 95 else '调整'}")
    print(f"{'R50-80(保守)':<20} {rate2:>6.1f}% {'':<6} {occ2:>6.4f}% {'':<6} {'✓ 适中' if 85 <= rate2 <= 95 else '调整'}")
    print(f"{'R60-90(激进)':<20} {rate3:>6.1f}% {'':<6} {occ3:>6.4f}% {'':<6} {'✓ 适中' if 85 <= rate3 <= 95 else '调整'}")
    
    print(f"\n{'='*70}")
    print("最终建议")
    print(f"{'='*70}")
    if 85 <= rate1 <= 95:
        print("✓ 当前配置(R55-85)合适，放置率和难度平衡良好")
    elif rate1 < 85:
        print("⚠ 当前配置(R55-85)难度偏高，建议使用保守配置(R50-80)")
    else:
        print("⚠ 当前配置(R55-85)难度偏低，建议使用激进配置(R60-90)")
    print(f"{'='*70}")
