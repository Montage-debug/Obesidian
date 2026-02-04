"""
PID采样控制器模块 - 移植自MATLAB版本
PID Sampling Controller for Dynamic Informed Subset Control

核心创新：
- 被控量：路径代价改进效率（而非距离误差）
- 控制输出1：gamma（椭球膨胀系数）- 控制知情子集体积
- 控制输出2：p_informed（知情采样概率）- 控制采样策略
- 抗饱和：完整的Anti-windup机制

参考文献：
- MATLAB版本：copilot-1.6/PIDSamplingController.m
- 基于"PID控制知情子集体积/采样概率的闭环调度"方案
"""

import numpy as np
from typing import Dict, Tuple, Optional


class PIDSamplingController:
    """
    PID采样控制器 - 动态调节知情采样子集
    
    核心原理：
    1. 测量量y_k：窗口内的路径代价改进率（平滑后）
    2. 误差e_k：目标效率 - 实际效率
    3. PID输出u_k控制gamma和p的变化：
       - 若改进缓慢（e_k>0）→ 增大gamma、减小p（增强探索）
       - 若改进顺利（e_k<0）→ 减小gamma、增大p（加强开发）
    """
    
    def __init__(
        self,
        Kp: float = 2.0,
        Ki: float = 0.2,
        Kd: float = 0.8,
        window_size: int = 50,
        target_efficiency: float = 0.02,
        # PID参数边界
        I_min: float = -3.0,
        I_max: float = 3.0,
        # 平滑参数
        rho_y: float = 0.9,  # 改进率平滑系数
        rho_d: float = 0.8,  # 微分平滑系数
        # gamma参数（放宽约束，避免过度收缩）
        gamma_0: float = 8.0,  # 提高基准值
        gamma_min: float = 3.0,  # 提高下限，避免过小
        gamma_max: float = 15.0,  # 大幅提高上限
        alpha_gamma: float = 0.3,  # 降低响应速度，避免过快收缩
        # p_informed参数（提高知情采样概率）
        p_0: float = 0.7,  # 提高基准值
        p_min: float = 0.4,  # 提高下限
        p_max: float = 0.9,  # 提高上限
        alpha_p: float = 0.2,  # 降低响应速度
        # 其他
        epsilon: float = 1e-6
    ):
        """
        初始化PID采样控制器
        
        Args:
            Kp: 比例增益
            Ki: 积分增益
            Kd: 微分增益
            window_size: 滑动窗口大小（计算改进效率）
            target_efficiency: 目标改进效率
            I_min, I_max: 积分饱和限制
            rho_y: 改进率EMA平滑系数
            rho_d: 微分项EMA平滑系数
            gamma_0: 初始椭球膨胀系数
            gamma_min, gamma_max: gamma边界
            alpha_gamma: gamma映射系数
            p_0: 初始知情采样概率
            p_min, p_max: p_informed边界
            alpha_p: p映射系数
            epsilon: 防除零小量
        """
        # PID增益
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        # 窗口和目标
        self.window_size = window_size
        self.target_efficiency = target_efficiency
        
        # 积分限制
        self.I_min = I_min
        self.I_max = I_max
        
        # 平滑系数
        self.rho_y = rho_y
        self.rho_d = rho_d
        
        # gamma参数
        self.gamma_0 = gamma_0
        self.gamma_min = gamma_min
        self.gamma_max = gamma_max
        self.alpha_gamma = alpha_gamma
        
        # p_informed参数
        self.p_0 = p_0
        self.p_min = p_min
        self.p_max = p_max
        self.alpha_p = alpha_p
        
        # 其他
        self.epsilon = epsilon
        
        # 状态变量
        self.cost_history = []  # 代价历史
        self.e_prev = 0.0  # 上次误差
        self.I = 0.0  # 积分项
        self.d_filt = 0.0  # 滤波微分项
        self.ybar = 0.0  # 平滑后的改进率
        self.iter_count = 0  # 迭代计数
        
        # 当前控制量
        self.current_gamma = gamma_0
        self.current_p_informed = 0.0  # 初始无解时不用知情采样
        self.current_u = 0.0
        self.current_y = 0.0
        self.current_error = 0.0
        
        # 诊断信息
        self.debug_info = {
            'P_term': 0.0,
            'I_term': 0.0,
            'D_term': 0.0,
            'saturated': False
        }
    
    def update(self, c_best: float) -> Tuple[float, float, Dict]:
        """
        更新PID控制器并返回控制量
        
        Args:
            c_best: 当前最优路径代价（Inf表示尚无可行解）
            
        Returns:
            gamma: 当前椭球膨胀系数 [gamma_min, gamma_max]
            p_informed: 当前知情采样概率 [p_min, p_max]
            info: 诊断信息字典
        """
        self.iter_count += 1
        self.cost_history.append(c_best)
        
        # ========== 初始阶段：尚无解或数据不足 ==========
        if np.isinf(c_best) or len(self.cost_history) < self.window_size + 1:
            # 使用最大探索策略
            gamma = self.gamma_max
            p_informed = 0.0
            
            info = {
                'gamma': gamma,
                'p_informed': p_informed,
                'u': 0.0,
                'error': 0.0,
                'y': 0.0,
                'stage': 'exploration',
                'P': 0.0,
                'I': 0.0,
                'D': 0.0
            }
            
            return gamma, p_informed, info
        
        # ========== 计算改进效率（测量量y_k） ==========
        c_old = self.cost_history[-(self.window_size + 1)]
        c_current = c_best
        
        # 计算改进率（有界、归一化）
        y_k = max(0, min(1, (c_old - c_current) / (c_old + self.epsilon)))
        
        # EMA平滑（减少噪声）
        self.ybar = self.rho_y * self.ybar + (1 - self.rho_y) * y_k
        
        # ========== PID误差计算 ==========
        e_k = self.target_efficiency - self.ybar
        
        # ========== 滤波微分（一阶低通滤波） ==========
        self.d_filt = self.rho_d * self.d_filt + (1 - self.rho_d) * (e_k - self.e_prev)
        
        # ========== 积分候选（带饱和限制） ==========
        I_cand = self.I + e_k
        I_cand = np.clip(I_cand, self.I_min, self.I_max)
        
        # ========== PID三项计算 ==========
        P_term = self.Kp * e_k
        I_term = self.Ki * I_cand
        D_term = self.Kd * self.d_filt
        
        u_cand = P_term + I_term + D_term
        
        # ========== 映射到gamma和p ==========
        # gamma：膨胀系数（探索越多越大）
        # 公式：gamma = gamma_0 * exp(alpha_gamma * u)
        gamma_cand = self.gamma_0 * np.exp(self.alpha_gamma * u_cand)
        gamma_cand = np.clip(gamma_cand, self.gamma_min, self.gamma_max)
        
        # p_informed：知情采样概率（探索越多越小）
        # ★★★ V5修复：反转符号，改进慢时降低p_informed ★★★
        # 逻辑：e_k<0 (改进慢) → u<0 → tanh(u)<0 → p↓ (减少知情采样，增加探索)
        #      e_k>0 (改进快) → u>0 → tanh(u)>0 → p↑ (增加知情采样)
        p_cand = self.p_0 + self.alpha_p * np.tanh(u_cand)  # 原来是减号，现在改为加号
        p_cand = np.clip(p_cand, self.p_min, self.p_max)
        
        # ========== 抗积分饱和（Anti-windup） ==========
        # 判断是否饱和：如果控制量达到边界且误差继续推向边界，则冻结积分
        saturated = False
        
        # gamma饱和检测
        if (gamma_cand >= self.gamma_max - 1e-6 and e_k > 0) or \
           (gamma_cand <= self.gamma_min + 1e-6 and e_k < 0):
            saturated = True
        
        # p饱和检测
        if (p_cand <= self.p_min + 1e-6 and e_k > 0) or \
           (p_cand >= self.p_max - 1e-6 and e_k < 0):
            saturated = True
        
        if saturated:
            # 冻结积分，重新计算u
            u_k = P_term + self.Ki * self.I + D_term
            
            gamma = self.gamma_0 * np.exp(self.alpha_gamma * u_k)
            gamma = np.clip(gamma, self.gamma_min, self.gamma_max)
            
            p_informed = self.p_0 - self.alpha_p * np.tanh(u_k)
            p_informed = np.clip(p_informed, self.p_min, self.p_max)
        else:
            # 更新积分
            self.I = I_cand
            gamma = gamma_cand
            p_informed = p_cand
            u_k = u_cand
        
        # ========== 更新状态 ==========
        self.e_prev = e_k
        self.current_gamma = gamma
        self.current_p_informed = p_informed
        self.current_u = u_k
        self.current_y = self.ybar
        self.current_error = e_k
        
        # 保存诊断信息
        self.debug_info = {
            'P_term': P_term,
            'I_term': I_term,
            'D_term': D_term,
            'saturated': saturated
        }
        
        # ========== 返回结果 ==========
        info = {
            'gamma': gamma,
            'p_informed': p_informed,
            'u': u_k,
            'error': e_k,
            'y': self.ybar,
            'y_raw': y_k,
            'stage': 'converging' if e_k < 0 else 'exploring',
            'P': P_term,
            'I': I_term,
            'D': D_term,
            'saturated': saturated,
            'c_best': c_best,
            'improvement_efficiency': self.ybar
        }
        
        return gamma, p_informed, info
    
    def reset(self):
        """重置PID状态"""
        self.cost_history = []
        self.e_prev = 0.0
        self.I = 0.0
        self.d_filt = 0.0
        self.ybar = 0.0
        self.iter_count = 0
        
        self.current_gamma = self.gamma_0
        self.current_p_informed = 0.0
        self.current_u = 0.0
        self.current_y = 0.0
        self.current_error = 0.0
    
    def get_current_state(self) -> Dict:
        """获取当前状态（用于可视化）"""
        return {
            'gamma': self.current_gamma,
            'p_informed': self.current_p_informed,
            'u': self.current_u,
            'error': self.current_error,
            'y': self.current_y,
            'I': self.I,
            'iter_count': self.iter_count,
            'cost_history_length': len(self.cost_history),
            **self.debug_info
        }


class AdaptivePIDGains:
    """
    自适应PID增益调整器（可选功能）
    
    根据迭代进度动态调整Kp, Ki, Kd参数
    参考MATLAB版本的adaptivePIDGains.m
    
    策略：
    - 探索阶段（前30%）：高Kp，强探索
    - 平衡阶段（30%-70%）：平衡参数
    - 收敛阶段（后30%）：高Kd，强阻尼
    """
    
    def __init__(
        self,
        Kp_base: float = 2.0,
        Ki_base: float = 0.2,
        Kd_base: float = 0.8,
        beta_p: float = 0.5,
        beta_i: float = 0.6,
        beta_d: float = 0.4
    ):
        """
        初始化自适应PID增益调整器
        
        Args:
            Kp_base, Ki_base, Kd_base: 基准PID增益
            beta_p, beta_i, beta_d: 调制幅度系数
        """
        self.Kp_base = Kp_base
        self.Ki_base = Ki_base
        self.Kd_base = Kd_base
        self.beta_p = beta_p
        self.beta_i = beta_i
        self.beta_d = beta_d
    
    def get_gains(self, iter_count: int, max_iterations: int) -> Tuple[float, float, float, str]:
        """
        获取当前迭代的自适应PID增益
        
        Args:
            iter_count: 当前迭代次数
            max_iterations: 最大迭代次数
            
        Returns:
            Kp, Ki, Kd: 自适应PID增益
            stage: 当前阶段 ('explore', 'balance', 'converge')
        """
        # 归一化进度 [0, 1]
        alpha = iter_count / max_iterations
        
        # 自适应公式（参考MATLAB版本）
        # Kp(α) = Kp_base × (1 + β_p × cos(π × α))
        Kp = self.Kp_base * (1 + self.beta_p * np.cos(np.pi * alpha))
        
        # Ki(α) = Ki_base × (1 - β_i × α²)
        Ki = self.Ki_base * (1 - self.beta_i * alpha**2)
        
        # Kd(α) = Kd_base × (1 + β_d × sin(π × α))
        Kd = self.Kd_base * (1 + self.beta_d * np.sin(np.pi * alpha))
        
        # 确定当前阶段
        if alpha < 0.3:
            stage = 'explore'
        elif alpha < 0.7:
            stage = 'balance'
        else:
            stage = 'converge'
        
        return Kp, Ki, Kd, stage
    
    def update_controller_gains(
        self,
        controller: PIDSamplingController,
        iter_count: int,
        max_iterations: int
    ):
        """
        更新控制器的PID增益
        
        Args:
            controller: PID采样控制器实例
            iter_count: 当前迭代次数
            max_iterations: 最大迭代次数
        """
        Kp, Ki, Kd, stage = self.get_gains(iter_count, max_iterations)
        controller.Kp = Kp
        controller.Ki = Ki
        controller.Kd = Kd
        return stage
