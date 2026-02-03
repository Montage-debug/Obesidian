#!/usr/bin/env python3
"""
快速测试：验证ESR修复后的行为
- 探索阶段(p_informed=0.0)应该ESR≈0%
- PID激活后ESR应该等于p_informed值
"""

import sys
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / 'src'))

from src.sc_rrt_basic_pid import SCRRTBasicPID

def test_esr_logic():
    """测试ESR统计逻辑是否正确"""
    print("=" * 60)
    print("测试ESR统计逻辑")
    print("=" * 60)
    
    # 3D场景，添加障碍物以确保需要多次迭代才能找到解
    start = np.array([75, 75, 75])
    goal = np.array([1425, 1425, 1425])
    bounds = [0, 1500, 0, 1500, 0, 1500]
    
    # 添加一些障碍物
    obstacles = []
    np.random.seed(42)
    for _ in range(50):  # 50个障碍物
        center = np.random.uniform(200, 1300, 3)
        radius = np.random.uniform(40, 60)
        obstacles.append({'center': center, 'radius': radius})
    
    env = {
        'start': start,
        'goal': goal,
        'bounds': bounds,
        'obstacles': obstacles,
        'dim': 3
    }
    
    # Low_PID配置
    planner = SCRRTBasicPID(
        env=env,
        Kp=0.1, Ki=0.015, Kd=0.04,
        max_iterations=800,  # 增加迭代次数
        mode='custom_pid',
        step_size=50.0,
        goal_threshold=50.0
    )
    
    # 运行规划
    path, tree, success, stats = planner.plan()
    
    # 打印PID历史信息以调试
    if stats.get('pid_sampling_history'):
        history = stats['pid_sampling_history']
        print(f"\n调试信息 - PID历史:")
        print(f"  迭代次数: {len(history['iterations'])}")
        if len(history['iterations']) > 0:
            print(f"  最后5次p_informed_A: {history['p_informed_A'][-5:]}")
            print(f"  最后5次p_informed_B: {history['p_informed_B'][-5:]}")
            print(f"  最后5次gamma_A: {history['gamma_A'][-5:]}")
    
    print(f"\n调试信息 - 属性:")
    print(f"  use_new_pid_controller: {planner.use_new_pid_controller}")
    print(f"  _p_informed_A: {getattr(planner, '_p_informed_A', 'NOT SET')}")
    print(f"  _p_informed_B: {getattr(planner, '_p_informed_B', 'NOT SET')}")
    
    # 分析结果
    print(f"\n规划结果:")
    print(f"  总采样次数: {stats['total_samples_attempted']}")
    print(f"  椭球内采样: {stats['samples_in_ellipsoid']}")
    print(f"  ESR: {stats['effective_sampling_ratio']*100:.2f}%")
    print(f"  成功: {success}")
    
    # 预期：如果没有找到可行解，p_informed=0，ESR应该≈0%
    # 如果找到了解，ESR应该等于平均p_informed值（Low_PID约30-40%）
    if success:
        print(f"\n✅ 找到解！ESR应该在20-50%范围")
        if 20 <= stats['effective_sampling_ratio']*100 <= 50:
            print(f"✅ ESR = {stats['effective_sampling_ratio']*100:.2f}% 在合理范围")
        else:
            print(f"⚠️ ESR = {stats['effective_sampling_ratio']*100:.2f}% 异常（预期20-50%）")
    else:
        print(f"\n未找到解，ESR应该≈0%")
        if stats['effective_sampling_ratio']*100 < 5:
            print(f"✅ ESR = {stats['effective_sampling_ratio']*100:.2f}% 正常（探索阶段p_informed=0）")
        else:
            print(f"⚠️ ESR = {stats['effective_sampling_ratio']*100:.2f}% 异常（预期<5%）")
    
    print("\n" + "=" * 60)

if __name__ == '__main__':
    test_esr_logic()
