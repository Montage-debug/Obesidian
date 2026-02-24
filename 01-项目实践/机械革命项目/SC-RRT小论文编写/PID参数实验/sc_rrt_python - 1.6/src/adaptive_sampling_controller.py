"""
自适应采样控制器 - 基于搜索状态反馈的在线调节机制
Adaptive Sampling Controller - Online Adjustment Based on Search State Feedback

核心思想：
不依赖PID的误差反馈，而是根据搜索状态（迭代进度、解的状态、树的规模等）
进行阶段性调节，符合RRT算法的随机性和阶段性特点。

创新点：
1. 三阶段自适应策略（探索→平衡→收敛）
2. 基于离散状态而非连续误差的调节
3. 有效采样率实时监控
4. 动态椭球约束参数
"""

import numpy as np
from typing import Dict, Tuple, Optional
from enum import Enum


class SearchPhase(Enum):
    """搜索阶段枚举"""
    EXPLORATION = "exploration"      # 探索阶段：无解，大范围搜索
    EXPLOITATION = "exploitation"    # 开发阶段：有解，精细搜索
    CONVERGENCE = "convergence"      # 收敛阶段：后期优化


class AdaptiveSamplingController:
    """
    自适应采样控制器
    
    核心机制：
    1. 根据迭代进度和搜索状态确定当前阶段
    2. 每个阶段有不同的采样策略（椭球大小、约束概率）
    3. 实时监控有效采样率，避免过度约束
    
    参数说明：
    - gamma: 椭球膨胀系数，控制知情子集的体积
    - p_informed: 知情采样概率，控制约束采样的频率
    - 有效采样率: 成功添加节点的比例（避免过多碰撞）
    """
    
    def __init__(
        self,
        max_iterations: int = 1000,
        # 探索阶段参数（无解时）
        gamma_explore: float = 6.0,
        p_explore: float = 0.2,
        # 开发阶段参数（有解后）
        gamma_exploit: float = 3.5,
        p_exploit: float = 0.5,
        # 收敛阶段参数（后期）
        gamma_converge: float = 2.0,
        p_converge: float = 0.7,
        # 阶段转换阈值
        exploit_iter_ratio: float = 0.3,    # 有解后，30%进度开始开发
        converge_iter_ratio: float = 0.7,   # 70%进度开始收敛
        # 有效采样率监控
        sample_efficiency_window: int = 50,
        min_efficiency_threshold: float = 0.2,  # 低于20%则放宽约束
        # gamma动态调整范围
        gamma_min: float = 1.5,
        gamma_max: float = 8.0
    ):
        """
        初始化自适应采样控制器
        
        Args:
            max_iterations: 最大迭代次数
            gamma_explore/exploit/converge: 三阶段椭球膨胀系数
            p_explore/exploit/converge: 三阶段知情采样概率
            exploit_iter_ratio: 开发阶段启动比例
            converge_iter_ratio: 收敛阶段启动比例
            sample_efficiency_window: 采样效率统计窗口
            min_efficiency_threshold: 最低效率阈值（触发放宽）
            gamma_min/max: gamma边界
        """
        self.max_iterations = max_iterations
        
        # 三阶段基准参数
        self.gamma_explore = gamma_explore
        self.p_explore = p_explore
        self.gamma_exploit = gamma_exploit
        self.p_exploit = p_exploit
        self.gamma_converge = gamma_converge
        self.p_converge = p_converge
        
        # 阶段转换
        self.exploit_iter_ratio = exploit_iter_ratio
        self.converge_iter_ratio = converge_iter_ratio
        
        # 采样效率监控
        self.sample_efficiency_window = sample_efficiency_window
        self.min_efficiency_threshold = min_efficiency_threshold
        self.recent_samples = []  # [(total, valid), ...]
        
        # 参数边界
        self.gamma_min = gamma_min
        self.gamma_max = gamma_max
        
        # 状态变量
        self.current_phase = SearchPhase.EXPLORATION
        self.iteration_count = 0
        self.has_solution = False
        self.solution_found_iter = None
        self.current_gamma = gamma_explore
        self.current_p_informed = p_explore
        
        # 统计信息
        self.phase_history = []
        self.gamma_history = []
        self.p_history = []
        self.efficiency_history = []
    
    def update(
        self,
        iteration: int,
        has_solution: bool,
        tree_size_a: int,
        tree_size_b: int,
        recent_valid_samples: int = 0,
        recent_total_samples: int = 0
    ) -> Tuple[float, float, Dict]:
        """
        更新采样控制参数
        
        Args:
            iteration: 当前迭代次数
            has_solution: 是否已找到可行解
            tree_size_a: 树A的节点数
            tree_size_b: 树B的节点数
            recent_valid_samples: 最近有效采样数
            recent_total_samples: 最近总采样数
            
        Returns:
            gamma: 椭球膨胀系数
            p_informed: 知情采样概率
            info: 调试信息
        """
        self.iteration_count = iteration
        
        # 更新解的状态
        if has_solution and not self.has_solution:
            self.has_solution = True
            self.solution_found_iter = iteration
        
        # 计算迭代进度
        progress = iteration / self.max_iterations
        
        # 更新采样效率
        if recent_total_samples > 0:
            self.recent_samples.append((recent_total_samples, recent_valid_samples))
            if len(self.recent_samples) > self.sample_efficiency_window:
                self.recent_samples.pop(0)
        
        # 计算当前采样效率
        sample_efficiency = self._calculate_sample_efficiency()
        
        # 确定当前搜索阶段
        phase = self._determine_phase(progress, has_solution)
        self.current_phase = phase
        
        # 根据阶段获取基准参数
        gamma_base, p_base = self._get_base_params(phase)
        
        # 根据采样效率动态调整
        gamma, p_informed = self._adjust_by_efficiency(
            gamma_base, p_base, sample_efficiency
        )
        
        # 边界限制
        gamma = np.clip(gamma, self.gamma_min, self.gamma_max)
        p_informed = np.clip(p_informed, 0.0, 1.0)
        
        # 保存状态
        self.current_gamma = gamma
        self.current_p_informed = p_informed
        
        # 记录历史
        self.phase_history.append(phase.value)
        self.gamma_history.append(gamma)
        self.p_history.append(p_informed)
        self.efficiency_history.append(sample_efficiency)
        
        # 构建调试信息
        info = {
            'phase': phase.value,
            'gamma': gamma,
            'p_informed': p_informed,
            'sample_efficiency': sample_efficiency,
            'progress': progress,
            'has_solution': has_solution,
            'tree_size_total': tree_size_a + tree_size_b,
            'gamma_base': gamma_base,
            'p_base': p_base,
            'adjustment': 'relaxed' if sample_efficiency < self.min_efficiency_threshold else 'normal'
        }
        
        return gamma, p_informed, info
    
    def _determine_phase(self, progress: float, has_solution: bool) -> SearchPhase:
        """
        确定当前搜索阶段
        
        逻辑：
        1. 无解且进度<30% → 探索
        2. 有解且进度<70% → 开发
        3. 进度≥70% → 收敛
        """
        if not has_solution:
            return SearchPhase.EXPLORATION
        
        if progress < self.converge_iter_ratio:
            return SearchPhase.EXPLOITATION
        else:
            return SearchPhase.CONVERGENCE
    
    def _get_base_params(self, phase: SearchPhase) -> Tuple[float, float]:
        """获取阶段对应的基准参数"""
        if phase == SearchPhase.EXPLORATION:
            return self.gamma_explore, self.p_explore
        elif phase == SearchPhase.EXPLOITATION:
            return self.gamma_exploit, self.p_exploit
        else:  # CONVERGENCE
            return self.gamma_converge, self.p_converge
    
    def _calculate_sample_efficiency(self) -> float:
        """
        计算采样效率
        
        Returns:
            efficiency: 有效采样率 [0, 1]
        """
        if len(self.recent_samples) == 0:
            return 0.5  # 初始默认值
        
        total_total = sum(s[0] for s in self.recent_samples)
        total_valid = sum(s[1] for s in self.recent_samples)
        
        if total_total == 0:
            return 0.5
        
        return total_valid / total_total
    
    def _adjust_by_efficiency(
        self,
        gamma_base: float,
        p_base: float,
        efficiency: float
    ) -> Tuple[float, float]:
        """
        根据采样效率动态调整参数
        
        策略：
        - 效率过低（<20%）→ 放宽约束（增大gamma，降低p）
        - 效率正常 → 使用基准参数
        - 效率很高（>60%）→ 可以增强约束
        """
        if efficiency < self.min_efficiency_threshold:
            # 采样效率过低，放宽约束
            scale = 1.5  # 放大椭球
            gamma = gamma_base * scale
            p_informed = p_base * 0.7  # 降低约束概率
            return gamma, p_informed
        
        elif efficiency > 0.6:
            # 采样效率很高，可以增强约束
            gamma = gamma_base * 0.9
            p_informed = min(1.0, p_base * 1.2)
            return gamma, p_informed
        
        else:
            # 效率正常，使用基准参数
            return gamma_base, p_base
    
    def reset(self):
        """重置控制器状态"""
        self.current_phase = SearchPhase.EXPLORATION
        self.iteration_count = 0
        self.has_solution = False
        self.solution_found_iter = None
        self.current_gamma = self.gamma_explore
        self.current_p_informed = self.p_explore
        self.recent_samples = []
        
        self.phase_history = []
        self.gamma_history = []
        self.p_history = []
        self.efficiency_history = []
    
    def get_current_state(self) -> Dict:
        """获取当前状态（用于可视化）"""
        return {
            'phase': self.current_phase.value,
            'gamma': self.current_gamma,
            'p_informed': self.current_p_informed,
            'iteration': self.iteration_count,
            'has_solution': self.has_solution,
            'sample_efficiency': self._calculate_sample_efficiency(),
            'phase_history': self.phase_history.copy(),
            'gamma_history': self.gamma_history.copy(),
            'p_history': self.p_history.copy(),
            'efficiency_history': self.efficiency_history.copy()
        }


class AdaptiveSamplingConfig:
    """
    预定义的自适应采样配置
    
    提供不同难度场景下的推荐配置
    """
    
    @staticmethod
    def get_config(scenario: str = "balanced") -> Dict:
        """
        获取预定义配置
        
        Args:
            scenario: 场景类型
                - "aggressive": 激进探索（低密度障碍物）
                - "balanced": 平衡策略（中等密度）
                - "conservative": 保守策略（高密度障碍物）
                
        Returns:
            config: 配置字典
        """
        configs = {
            "aggressive": {
                "gamma_explore": 7.0,
                "p_explore": 0.15,
                "gamma_exploit": 4.0,
                "p_exploit": 0.4,
                "gamma_converge": 2.5,
                "p_converge": 0.6,
                "min_efficiency_threshold": 0.15
            },
            "balanced": {
                "gamma_explore": 6.0,
                "p_explore": 0.2,
                "gamma_exploit": 3.5,
                "p_exploit": 0.5,
                "gamma_converge": 2.0,
                "p_converge": 0.7,
                "min_efficiency_threshold": 0.2
            },
            "conservative": {
                "gamma_explore": 5.0,
                "p_explore": 0.25,
                "gamma_exploit": 3.0,
                "p_exploit": 0.55,
                "gamma_converge": 1.8,
                "p_converge": 0.75,
                "min_efficiency_threshold": 0.25
            }
        }
        
        return configs.get(scenario, configs["balanced"])
