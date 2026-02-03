"""
测试PID采样控制器参数是否正确应用
"""
import numpy as np
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent / 'src'))

from src.environment import EnvironmentConfig
from src.sc_rrt_basic_pid import SCRRTBasicPID
from src.pid_sampling_controller import PIDSamplingController

print("="*70)
print("测试PID采样控制器参数")
print("="*70)

# 测试控制器默认参数
print("\n1. 测试PIDSamplingController默认参数:")
controller = PIDSamplingController(Kp=0.2, Ki=0.03, Kd=0.08)
print(f"   p_0 = {controller.p_0}")
print(f"   p_min = {controller.p_min}")
print(f"   p_max = {controller.p_max}")
print(f"   gamma_0 = {controller.gamma_0}")
print(f"   gamma_min = {controller.gamma_min}")
print(f"   gamma_max = {controller.gamma_max}")

# 测试显式传参
print("\n2. 测试显式传递V3参数:")
controller_v3 = PIDSamplingController(
    Kp=0.2, Ki=0.03, Kd=0.08,
    p_0=0.3, p_min=0.1, p_max=0.6,
    gamma_0=3.0, gamma_min=1.5, gamma_max=6.0
)
print(f"   p_0 = {controller_v3.p_0}")
print(f"   p_min = {controller_v3.p_min}")
print(f"   p_max = {controller_v3.p_max}")
print(f"   gamma_0 = {controller_v3.gamma_0}")
print(f"   gamma_min = {controller_v3.gamma_min}")
print(f"   gamma_max = {controller_v3.gamma_max}")

# 测试实际规划中的p_informed值
print("\n3. 测试实际规划中的p_informed值:")
env = EnvironmentConfig.generate_2d_environment(
    bounds=[0, 1000, 0, 1000],
    num_obstacles=50,
    start_point=np.array([50, 50]),
    goal_point=np.array([950, 950]),
    radius_range=(15, 30),
    seed=42
)

planner = SCRRTBasicPID(
    env=env,
    max_iterations=100,
    mode='custom_pid',
    Kp=0.20, Ki=0.03, Kd=0.08,
    verbose=False
)

# 检查控制器是否正确创建
if planner.pid_sampling_controller_A:
    print(f"   控制器A已创建")
    print(f"   p_0 = {planner.pid_sampling_controller_A.p_0}")
    print(f"   p_max = {planner.pid_sampling_controller_A.p_max}")
    print(f"   gamma_max = {planner.pid_sampling_controller_A.gamma_max}")
    
    # 运行几次更新看p_informed的值
    print(f"\n   模拟PID更新:")
    for i in range(5):
        c_best = 2000 - i * 100  # 模拟代价递减
        gamma, p_informed, info = planner.pid_sampling_controller_A.update(c_best)
        print(f"   迭代{i+1}: c_best={c_best:.1f}, gamma={gamma:.2f}, p_informed={p_informed:.3f}, stage={info.get('stage', 'N/A')}")
else:
    print("   ❌ 控制器未创建!")

print("\n" + "="*70)
