"""
自动寻找最佳三维障碍物配置
目标：放置率85%+，难度适中（成功率预期40-70%）
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent / 'src'))

import numpy as np
from src.environment import EnvironmentConfig

def evaluate_config(radius_min, radius_max, min_spacing, clearance, num_obstacles=400, seed=302):
    """评估配置的可行性"""
    env = EnvironmentConfig.generate_3d_environment(
        bounds=[0, 1500, 0, 1500, 0, 1500],
        num_obstacles=num_obstacles,
        start_point=np.array([75, 75, 75]),
        goal_point=np.array([1425, 1425, 1425]),
        radius_range=(radius_min, radius_max),
        min_spacing=min_spacing,
        clearance=clearance,
        seed=seed
    )
    
    actual = len(env['obstacles'])
    placement_rate = actual / num_obstacles * 100
    
    # 计算体积占比
    total_volume = 0
    for obs in env['obstacles']:
        radius = obs[3]
        total_volume += (4/3) * np.pi * radius**3
    
    space_volume = 1500 * 1500 * 1500
    occupancy_rate = total_volume / space_volume * 100
    
    avg_radius = np.mean(env['obstacles'][:, 3]) if actual > 0 else 0
    
    # 评分：放置率85%+最好，占用率在5-12%最佳
    score = 0
    if placement_rate >= 95:
        score += 100
    elif placement_rate >= 90:
        score += 90
    elif placement_rate >= 85:
        score += 80
    elif placement_rate >= 80:
        score += 60
    else:
        score += max(0, placement_rate)
    
    # 占用率评分（目标5-12%，体现PID优势但不至于太难）
    if 6 <= occupancy_rate <= 10:
        score += 100
    elif 5 <= occupancy_rate <= 12:
        score += 80
    elif 4 <= occupancy_rate <= 14:
        score += 60
    elif 3 <= occupancy_rate <= 16:
        score += 40
    else:
        score += max(0, 50 - abs(occupancy_rate - 9) * 5)
    
    return {
        'radius_min': radius_min,
        'radius_max': radius_max,
        'min_spacing': min_spacing,
        'clearance': clearance,
        'placement_rate': placement_rate,
        'occupancy_rate': occupancy_rate,
        'avg_radius': avg_radius,
        'actual_count': actual,
        'score': score
    }


def print_result(result, label=""):
    """打印配置结果"""
    print(f"\n{'='*70}")
    print(f"{label}")
    print(f"{'='*70}")
    print(f"半径范围: R{result['radius_min']:.0f}-{result['radius_max']:.0f}")
    print(f"间距/安全: {result['min_spacing']:.0f}/{result['clearance']:.0f}")
    print(f"放置率: {result['placement_rate']:.1f}% ({result['actual_count']}/400)")
    print(f"平均半径: {result['avg_radius']:.2f}")
    print(f"空间占用: {result['occupancy_rate']:.4f}%")
    print(f"综合得分: {result['score']:.0f}/200")
    
    # 评级
    if result['score'] >= 180:
        grade = "优秀 ★★★★★"
    elif result['score'] >= 160:
        grade = "良好 ★★★★"
    elif result['score'] >= 140:
        grade = "中等 ★★★"
    elif result['score'] >= 120:
        grade = "及格 ★★"
    else:
        grade = "不合格 ★"
    
    print(f"评级: {grade}")


if __name__ == '__main__':
    print("\n" + "="*70)
    print("自动寻找最佳三维障碍物配置")
    print("="*70)
    print("\n目标：")
    print("  • 放置率 ≥ 85%")
    print("  • 空间占用率 6-10%（难度适中）")
    print("  • 能体现PID优势（成功率差异20%+）")
    
    # 候选配置列表
    configs = []
    
    # 测试1：保守配置（小障碍物）
    print("\n\n【测试1/8】保守配置 - 小障碍物")
    result = evaluate_config(
        radius_min=45, radius_max=70,
        min_spacing=18, clearance=22
    )
    configs.append(result)
    print_result(result, "保守配置 (R45-70)")
    
    # 测试2：中等偏小
    print("\n\n【测试2/8】中等偏小")
    result = evaluate_config(
        radius_min=48, radius_max=75,
        min_spacing=19, clearance=23
    )
    configs.append(result)
    print_result(result, "中等偏小 (R48-75)")
    
    # 测试3：中等
    print("\n\n【测试3/8】中等配置")
    result = evaluate_config(
        radius_min=50, radius_max=78,
        min_spacing=20, clearance=24
    )
    configs.append(result)
    print_result(result, "中等配置 (R50-78)")
    
    # 测试4：中等偏大
    print("\n\n【测试4/8】中等偏大")
    result = evaluate_config(
        radius_min=52, radius_max=80,
        min_spacing=20, clearance=24
    )
    configs.append(result)
    print_result(result, "中等偏大 (R52-80)")
    
    # 测试5：当前配置
    print("\n\n【测试5/8】当前配置")
    result = evaluate_config(
        radius_min=55, radius_max=85,
        min_spacing=22, clearance=25
    )
    configs.append(result)
    print_result(result, "当前配置 (R55-85)")
    
    # 测试6：适中（基于二维成功经验）
    print("\n\n【测试6/8】适中配置（基于二维比例）")
    result = evaluate_config(
        radius_min=50, radius_max=75,
        min_spacing=18, clearance=22
    )
    configs.append(result)
    print_result(result, "适中配置 (R50-75)")
    
    # 测试7：平衡配置
    print("\n\n【测试7/8】平衡配置")
    result = evaluate_config(
        radius_min=48, radius_max=72,
        min_spacing=18, clearance=22
    )
    configs.append(result)
    print_result(result, "平衡配置 (R48-72)")
    
    # 测试8：精细配置
    print("\n\n【测试8/8】精细配置")
    result = evaluate_config(
        radius_min=46, radius_max=68,
        min_spacing=17, clearance=21
    )
    configs.append(result)
    print_result(result, "精细配置 (R46-68)")
    
    # 排序并推荐
    configs.sort(key=lambda x: x['score'], reverse=True)
    
    print("\n\n" + "="*70)
    print("配置对比总结（按得分排序）")
    print("="*70)
    print(f"\n{'排名':<6} {'配置':<18} {'放置率':<12} {'占用率':<12} {'得分':<8} {'推荐'}")
    print("-"*70)
    
    for i, cfg in enumerate(configs, 1):
        recommend = "★★★推荐" if i == 1 else ("★★备选" if i <= 3 else "")
        print(f"{i:<6} R{cfg['radius_min']:.0f}-{cfg['radius_max']:.0f} "
              f"{'':<10} {cfg['placement_rate']:>6.1f}% {'':<6} "
              f"{cfg['occupancy_rate']:>6.4f}% {'':<6} "
              f"{cfg['score']:>6.0f} {'':<6} {recommend}")
    
    # 最佳配置详情
    best = configs[0]
    print("\n" + "="*70)
    print("🏆 最佳配置推荐")
    print("="*70)
    print(f"""
配置参数：
  radius_range=({best['radius_min']:.0f}, {best['radius_max']:.0f})
  min_spacing={best['min_spacing']:.0f}
  clearance={best['clearance']:.0f}

性能指标：
  放置率: {best['placement_rate']:.1f}% ({best['actual_count']}/400)
  平均半径: {best['avg_radius']:.2f}
  空间占用: {best['occupancy_rate']:.4f}%
  综合得分: {best['score']:.0f}/200

预期效果：
  • 障碍物能成功放置（放置率{best['placement_rate']:.0f}%）
  • 难度适中（占用率{best['occupancy_rate']:.1f}%）
  • PID优势明显（预计成功率差异20-30%）
""")
    
    print("="*70)
    print("代码更新建议")
    print("="*70)
    print(f"""
在 run_pid_experiment.py 中更新：

radius_range=({best['radius_min']:.0f}, {best['radius_max']:.0f}),  # ★最佳配置
min_spacing={best['min_spacing']:.0f},
clearance={best['clearance']:.0f},
""")
    print("="*70)
