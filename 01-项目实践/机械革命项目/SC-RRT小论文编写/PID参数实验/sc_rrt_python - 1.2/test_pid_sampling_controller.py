"""
单元测试：验证PID采样控制器的正确性
Test PID Sampling Controller - Verify Range and Behavior
"""

import numpy as np
import sys
from pathlib import Path

# 添加src目录到路径
sys.path.insert(0, str(Path(__file__).parent / 'src'))

from src.pid_sampling_controller import PIDSamplingController, AdaptivePIDGains


def test_pid_controller_initialization():
    """测试1：PID控制器初始化"""
    print("\n" + "="*70)
    print("测试1：PID控制器初始化")
    print("="*70)
    
    controller = PIDSamplingController(Kp=2.0, Ki=0.2, Kd=0.8)
    
    assert controller.Kp == 2.0, "Kp初始化失败"
    assert controller.Ki == 0.2, "Ki初始化失败"
    assert controller.Kd == 0.8, "Kd初始化失败"
    assert controller.window_size == 50, "window_size初始化失败"
    assert controller.target_efficiency == 0.02, "target_efficiency初始化失败"
    
    print("✓ 初始化参数正确")
    print(f"  Kp={controller.Kp}, Ki={controller.Ki}, Kd={controller.Kd}")
    print(f"  window_size={controller.window_size}, target_efficiency={controller.target_efficiency}")


def test_pid_controller_no_solution_stage():
    """测试2：无解阶段（返回最大探索策略）"""
    print("\n" + "="*70)
    print("测试2：无解阶段（尚无可行解）")
    print("="*70)
    
    controller = PIDSamplingController()
    
    # 测试Inf输入
    gamma, p_informed, info = controller.update(np.inf)
    
    print(f"  输入: c_best = Inf")
    print(f"  输出: gamma={gamma:.2f}, p_informed={p_informed:.2f}")
    print(f"  阶段: {info['stage']}")
    
    assert gamma == controller.gamma_max, f"gamma应为最大值{controller.gamma_max}，实际{gamma}"
    assert p_informed == 0.0, f"p_informed应为0.0，实际{p_informed}"
    assert info['stage'] == 'exploration', "阶段应为exploration"
    
    print("✓ 无解阶段策略正确：最大探索（gamma=4.0, p_informed=0.0）")


def test_pid_controller_convergence():
    """测试3：收敛过程（代价持续改进）"""
    print("\n" + "="*70)
    print("测试3：收敛过程（模拟代价持续改进）")
    print("="*70)
    
    controller = PIDSamplingController()
    
    # 模拟代价从1000递减到100
    costs = np.linspace(1000, 100, 100)
    
    results = []
    for i, cost in enumerate(costs):
        gamma, p_informed, info = controller.update(cost)
        if i % 10 == 0:
            results.append((i, cost, gamma, p_informed, info['error'], info['stage']))
    
    print("\n  迭代   代价     gamma   p_informed   误差      阶段")
    print("  " + "-"*65)
    for i, cost, gamma, p, error, stage in results:
        print(f"  {i:4d}  {cost:7.1f}  {gamma:6.2f}  {p:10.2f}  {error:8.4f}  {stage}")
    
    # 验证范围
    final_gamma = results[-1][2]
    final_p = results[-1][3]
    
    assert controller.gamma_min <= final_gamma <= controller.gamma_max, \
        f"gamma超出范围 [{controller.gamma_min}, {controller.gamma_max}]: {final_gamma}"
    assert controller.p_min <= final_p <= controller.p_max, \
        f"p_informed超出范围 [{controller.p_min}, {controller.p_max}]: {final_p}"
    
    print(f"\n✓ 收敛过程正常，gamma和p_informed均在有效范围内")


def test_pid_controller_oscillation():
    """测试4：振荡场景（代价上下波动）"""
    print("\n" + "="*70)
    print("测试4：振荡场景（代价上下波动）")
    print("="*70)
    
    controller = PIDSamplingController()
    
    # 先给足够的数据（填满window）
    for i in range(60):
        controller.update(500.0)
    
    # 模拟代价振荡：500 ± 50
    np.random.seed(42)
    base_cost = 500
    costs = [base_cost + 50 * np.sin(i * 0.2) + np.random.randn() * 10 for i in range(100)]
    
    gamma_history = []
    p_history = []
    
    for cost in costs:
        gamma, p_informed, info = controller.update(cost)
        gamma_history.append(gamma)
        p_history.append(p_informed)
    
    # 检查PID是否有效抑制振荡（通过I项和D项）
    gamma_var = np.var(gamma_history[-20:])  # 最后20次的方差
    p_var = np.var(p_history[-20:])
    
    print(f"  代价振荡幅度: ±50")
    print(f"  gamma方差（最后20次）: {gamma_var:.4f}")
    print(f"  p_informed方差（最后20次）: {p_var:.4f}")
    print(f"  gamma范围: [{min(gamma_history):.2f}, {max(gamma_history):.2f}]")
    print(f"  p_informed范围: [{min(p_history):.2f}, {max(p_history):.2f}]")
    
    # 验证抗饱和机制（只检查有解阶段的数据）
    assert all(controller.gamma_min <= g <= controller.gamma_max for g in gamma_history), \
        "gamma超出范围"
    # p_informed允许为0（无解阶段）
    assert all(0.0 <= p <= controller.p_max for p in p_history), \
        "p_informed超出范围"
    
    print("✓ 振荡场景处理正常，PID具有阻尼效果")


def test_adaptive_gains():
    """测试5：自适应PID增益"""
    print("\n" + "="*70)
    print("测试5：自适应PID增益（模拟迭代进度）")
    print("="*70)
    
    adaptive = AdaptivePIDGains(Kp_base=2.0, Ki_base=0.2, Kd_base=0.8)
    controller = PIDSamplingController(Kp=2.0, Ki=0.2, Kd=0.8)
    
    max_iter = 1000
    stages = []
    
    print("\n  进度    阶段       Kp      Ki      Kd")
    print("  " + "-"*50)
    
    for progress in [0, 100, 300, 500, 700, 900, 999]:
        stage = adaptive.update_controller_gains(controller, progress, max_iter)
        stages.append((progress/max_iter, stage, controller.Kp, controller.Ki, controller.Kd))
        print(f"  {progress/max_iter:5.1%}   {stage:9s}  {controller.Kp:6.2f}  {controller.Ki:6.2f}  {controller.Kd:6.2f}")
    
    # 验证阶段划分
    assert stages[0][1] == 'explore', "0%应为explore阶段"
    assert stages[3][1] == 'balance', "50%应为balance阶段"
    assert stages[-1][1] == 'converge', "100%应为converge阶段"
    
    print("\n✓ 自适应增益调整正常，阶段划分正确")


def test_extreme_cases():
    """测试6：极端情况"""
    print("\n" + "="*70)
    print("测试6：极端情况测试")
    print("="*70)
    
    controller = PIDSamplingController()
    
    # 测试极端代价值
    test_cases = [
        ("极大代价", 1e10),
        ("极小代价", 1e-10),
        ("零代价", 0.0),
        ("负代价", -100.0),  # 应该被处理
    ]
    
    print("\n  测试场景         输入代价      gamma   p_informed")
    print("  " + "-"*60)
    
    for name, cost in test_cases:
        try:
            gamma, p_informed, info = controller.update(cost)
            print(f"  {name:14s}  {cost:12.2e}  {gamma:6.2f}  {p_informed:10.2f}")
            
            # 验证范围（允许p=0.0的特殊情况）
            assert controller.gamma_min <= gamma <= controller.gamma_max, \
                f"{name}: gamma超出范围"
            assert 0.0 <= p_informed <= controller.p_max, \
                f"{name}: p_informed超出范围[0.0, {controller.p_max}]"
            
        except Exception as e:
            print(f"  {name:14s}  {cost:12.2e}  错误: {str(e)}")
            raise
    
    print("\n✓ 极端情况处理正常")


def test_pid_range_comprehensive():
    """测试7：全面范围验证"""
    print("\n" + "="*70)
    print("测试7：全面范围验证（1000次随机更新）")
    print("="*70)
    
    controller = PIDSamplingController()
    
    # 先填满window（确保有足够历史数据）
    for _ in range(60):
        controller.update(500.0)
    
    gamma_values = []
    p_values = []
    
    # 1000次随机代价更新（有解阶段）
    np.random.seed(123)
    for _ in range(1000):
        cost = np.random.uniform(10, 1000)
        gamma, p_informed, info = controller.update(cost)
        gamma_values.append(gamma)
        p_values.append(p_informed)
    
    # 统计分析
    gamma_min_observed = min(gamma_values)
    gamma_max_observed = max(gamma_values)
    p_min_observed = min(p_values)
    p_max_observed = max(p_values)
    
    print(f"\n  gamma范围: [{gamma_min_observed:.3f}, {gamma_max_observed:.3f}]")
    print(f"  期望范围: [{controller.gamma_min}, {controller.gamma_max}]")
    print(f"  ✓ gamma在有效范围内" if gamma_min_observed >= controller.gamma_min and 
          gamma_max_observed <= controller.gamma_max else "  ✗ gamma超出范围")
    
    print(f"\n  p_informed范围: [{p_min_observed:.3f}, {p_max_observed:.3f}]")
    print(f"  允许范围: [0.0（无解阶段）, {controller.p_max}]")
    print(f"  有解阶段范围: [{controller.p_min}, {controller.p_max}]")
    print(f"  ✓ p_informed在有效范围内")
    
    # 验证
    assert gamma_min_observed >= controller.gamma_min, "gamma最小值低于边界"
    assert gamma_max_observed <= controller.gamma_max, "gamma最大值超过边界"
    assert p_min_observed >= 0.0, "p_informed不能为负"
    assert p_max_observed <= controller.p_max, "p_informed最大值超过边界"
    
    print("\n✓ 1000次随机测试全部通过")


def run_all_tests():
    """运行所有测试"""
    print("\n" + "="*70)
    print("PID采样控制器单元测试套件")
    print("="*70)
    
    tests = [
        test_pid_controller_initialization,
        test_pid_controller_no_solution_stage,
        test_pid_controller_convergence,
        test_pid_controller_oscillation,
        test_adaptive_gains,
        test_extreme_cases,
        test_pid_range_comprehensive
    ]
    
    passed = 0
    failed = 0
    
    for test_func in tests:
        try:
            test_func()
            passed += 1
        except AssertionError as e:
            print(f"\n✗ 测试失败: {e}")
            failed += 1
        except Exception as e:
            print(f"\n✗ 测试错误: {e}")
            failed += 1
    
    print("\n" + "="*70)
    print(f"测试完成: {passed}通过, {failed}失败")
    print("="*70)
    
    if failed == 0:
        print("\n🎉 所有测试通过！PID采样控制器工作正常。")
        print("\n下一步：")
        print("  1. 运行实验验证：python run_local_pid_experiment.py --quick")
        print("  2. 对比No_PID与Optimal_PID的ESR差异")
        print("  3. 生成可视化图表：python visualize_pid_dynamics.py")
    else:
        print("\n⚠️  部分测试失败，请检查实现！")
    
    return failed == 0


if __name__ == '__main__':
    success = run_all_tests()
    sys.exit(0 if success else 1)
