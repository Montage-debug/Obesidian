"""
快速测试脚本
用于验证安装和算法功能
"""

import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent / 'src'))

import numpy as np
from src.environment import EnvironmentConfig
from src.sc_rrt_basic_pid import SCRRTBasicPID

def test_environment():
    """测试环境生成"""
    print("测试1: 环境生成")
    env = EnvironmentConfig.generate_2d_environment(
        bounds=[0, 100, 0, 100],
        num_obstacles=5,
        seed=42
    )
    print(f"  ✓ 生成2D环境: {env['num']}个障碍物")
    print(f"  起点: {env['start']}")
    print(f"  终点: {env['goal']}")
    return env

def test_planning(env):
    """测试路径规划"""
    print("\n测试2: 路径规划")
    planner = SCRRTBasicPID(
        env=env,
        max_iterations=500,
        Kp=0.25,
        Ki=0.04,
        Kd=0.10,
        verbose=False
    )
    
    print("  执行规划...")
    path, tree, success, metrics = planner.plan()
    
    if success:
        print(f"  ✓ 规划成功!")
        print(f"  路径长度: {metrics['path_length']:.2f}")
        print(f"  规划时间: {metrics['planning_time']:.2f}s")
        print(f"  迭代次数: {metrics['iterations']}")
    else:
        print(f"  ✗ 规划失败")
    
    return success

def main():
    print("=" * 60)
    print("SC-RRT Python 实现 - 快速测试")
    print("=" * 60)
    print()
    
    try:
        # 测试环境生成
        env = test_environment()
        
        # 测试路径规划
        success = test_planning(env)
        
        print()
        print("=" * 60)
        if success:
            print("✓ 所有测试通过!")
            print("系统运行正常，可以开始实验")
        else:
            print("⚠ 测试未全部通过，请检查配置")
        print("=" * 60)
        
    except Exception as e:
        print()
        print("=" * 60)
        print(f"✗ 测试失败: {str(e)}")
        print("=" * 60)
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
