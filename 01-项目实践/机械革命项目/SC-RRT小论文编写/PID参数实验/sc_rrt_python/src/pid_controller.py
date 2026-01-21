"""
PID权重控制器模块
PID Weight Controller for Adaptive Cost Function Weighting
"""

import numpy as np
from typing import Dict, Tuple


class PIDWeightController:
    """
    PID自适应代价函数权重控制器
    
    核心创新：
    - 被控量(PV): 知情子集有效采样率 + 双树靠近速度
    - 控制输出: 代价函数权重的参数化调整
    - 更新频率: 低频慢调节(K_pid = 20~50)
    """
    
    def __init__(
        self,
        Kp: float = 1.5,
        Ki: float = 0.05,
        Kd: float = 0.4,
        K_pid: int = 250,
        r_inf_target: Tuple[float, float] = (0.15, 0.30),
        alpha: float = 1.0,
        beta: float = 0.3
    ):
        """
        初始化PID控制器
        
        Args:
            Kp: 比例增益
            Ki: 积分增益
            Kd: 微分增益
            K_pid: PID更新间隔
            r_inf_target: 目标采样率区间 [min, max]
            alpha: 采样率权重
            beta: 靠近速度权重
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.K_pid = max(K_pid, 50)  # 最小50
        self.r_inf_target = r_inf_target
        self.alpha = alpha
        self.beta = beta
        
        # PID状态
        self.theta_L = 0.0  # 长度权重参数
        self.theta_C = 0.0  # 安全权重参数
        self.theta_S = 0.0  # 平滑权重参数
        self.integral_term = 0.0
        self.prev_error = 0.0
        
        # 参数边界
        self.theta_min = -2.0
        self.theta_max = 2.0
        
        # 积分饱和限制
        self.max_integral = 5.0
    
    def update(
        self,
        metrics: Dict,
        path_found: bool = False
    ) -> Tuple[np.ndarray, Dict]:
        """
        更新权重
        
        Args:
            metrics: 当前规划指标
                - r_inf: 知情子集有效采样率 [0,1]
                - v_meet: 双树靠近速度
                - d_min: 双树最小距离
            path_found: 是否已找到路径
            
        Returns:
            weights: 归一化权重 [w_L, w_C, w_S]
            control_info: 控制信息字典
        """
        # 1. 计算PID误差
        r_inf_target_center = np.mean(self.r_inf_target)
        e_r_inf = r_inf_target_center - metrics.get('r_inf', 0.0)
        
        # 靠近速度误差（期望 v_meet > 0）
        v_meet_target = 0.0
        e_v_meet = max(0, v_meet_target - metrics.get('v_meet', 0.0))
        
        # 综合误差（加权）
        error_total = self.alpha * e_r_inf + self.beta * e_v_meet
        
        # 2. PID控制计算
        # P项（比例控制）
        P_term = self.Kp * error_total
        
        # I项（积分控制，防饱和）
        new_integral = self.integral_term + error_total
        new_integral = np.clip(new_integral, -self.max_integral, self.max_integral)
        I_term = self.Ki * new_integral
        
        # D项（微分控制）
        error_derivative = error_total - self.prev_error
        D_term = self.Kd * error_derivative
        
        # PID输出
        u = P_term + I_term + D_term
        
        # 3. 路径状态自适应衰减
        if path_found:
            decay_factor = 0.9
            u = u * decay_factor
        
        # 4. 权重参数更新
        k_C = 0.5  # 安全权重调节系数
        k_S = 0.3  # 平滑权重调节系数
        
        # 更新参数（有界约束）
        theta_C_new = np.clip(
            self.theta_C + k_C * u,
            self.theta_min,
            self.theta_max
        )
        theta_S_new = np.clip(
            self.theta_S + k_S * u,
            self.theta_min,
            self.theta_max
        )
        theta_L_new = np.clip(
            self.theta_L - (k_C + k_S) * u,
            self.theta_min,
            self.theta_max
        )
        
        # 5. 归一化权重计算
        Z = np.exp(theta_L_new) + np.exp(theta_C_new) + np.exp(theta_S_new)
        w_L = np.exp(theta_L_new) / Z
        w_C = np.exp(theta_C_new) / Z
        w_S = np.exp(theta_S_new) / Z
        
        # 更新状态
        self.theta_L = theta_L_new
        self.theta_C = theta_C_new
        self.theta_S = theta_S_new
        self.integral_term = new_integral
        self.prev_error = error_total
        
        # 返回结果
        weights = np.array([w_L, w_C, w_S])
        
        control_info = {
            'error_total': error_total,
            'P_term': P_term,
            'I_term': I_term,
            'D_term': D_term,
            'u': u,
            'theta_L': theta_L_new,
            'theta_C': theta_C_new,
            'theta_S': theta_S_new,
            'weights': weights
        }
        
        return weights, control_info
    
    def reset(self):
        """重置PID状态"""
        self.theta_L = 0.0
        self.theta_C = 0.0
        self.theta_S = 0.0
        self.integral_term = 0.0
        self.prev_error = 0.0
