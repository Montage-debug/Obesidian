"""
SC-RRT 算法实现（基于搜索状态反馈的自适应采样控制）

Self-Constrained RRT with Adaptive Sampling Control

核心特点:
1. 双向RRT with 动态椭球约束采样
2. 动态交汇点转移机制
3. 基于搜索状态反馈的三阶段自适应控制
4. 支持对比实验（有/无自适应控制器）
"""

import numpy as np
import time
from typing import Dict, Optional, Tuple, List
from .geometry import (
    is_collision_free, steer_point, sample_point,
    sample_in_ellipsoid, calculate_path_length,
    calculate_path_smoothness, find_nearest_node
)
try:
    from .adaptive_sampling_controller import AdaptiveSamplingController
except ImportError:
    AdaptiveSamplingController = None


class SCRRTAdaptive:
    """SC-RRT 算法类（支持自适应采样控制）"""
    
    # 步长配置
    STEP_SIZE_RATIO = 0.005
    STEP_SIZE_MIN_2D = 8.0
    STEP_SIZE_MAX_2D = 70.0
    STEP_SIZE_MIN_3D = 15.0
    STEP_SIZE_MAX_3D = 50.0
    
    def __init__(
        self,
        env: Dict,
        max_iterations: int = 5000,
        mode: str = 'adaptive',
        step_size: Optional[float] = None,
        goal_threshold: Optional[float] = None,
        update_interval: int = 50,
        smoothing_factor: float = 0.7,
        ellipsoid_buffer: float = 1.2,
        use_pareto: bool = True,
        pareto_interval: int = 100,
        pareto_prob: float = 0.8,
        # 自适应控制器配置
        adaptive_config: Optional[Dict] = None,
        verbose: bool = True
    ):
        """
        初始化SC-RRT
        
        Args:
            env: 环境字典
            max_iterations: 最大迭代次数
            mode: 控制模式 ('adaptive'=使用自适应控制器 | 'no_adaptive'=不使用)
            step_size: 步长（None则自动设置）
            goal_threshold: 目标阈值（None则自动设置）
            update_interval: 交汇点更新间隔
            smoothing_factor: 交汇点平滑因子
            ellipsoid_buffer: 椭球体缓冲系数
            use_pareto: 是否使用Pareto前沿
            adaptive_config: 自适应控制器配置（None则使用默认）
            verbose: 是否打印详细信息
        """
        self.env = env
        self.max_iterations = max_iterations
        self.mode = mode
        self.update_interval = update_interval
        self.smoothing_factor = smoothing_factor
        self.ellipsoid_buffer = ellipsoid_buffer
        self.use_pareto = use_pareto
        self.pareto_interval = pareto_interval
        self.pareto_prob = pareto_prob
        self.verbose = verbose
        
        # 详细指标跟踪
        self.track_detailed_metrics = True
        
        # Pareto节点选择策略计数器
        self.pareto_counter = 0
        self.pareto_selections = []
        
        # ★★★ 新增：自适应采样控制器 ★★★
        self.use_adaptive_controller = (mode == 'adaptive' and AdaptiveSamplingController is not None)
        if self.use_adaptive_controller:
            # 使用配置或默认值
            if adaptive_config is None:
                adaptive_config = {
                    "gamma_explore": 6.0,
                    "p_explore": 0.2,
                    "gamma_exploit": 3.5,
                    "p_exploit": 0.5,
                    "gamma_converge": 2.0,
                    "p_converge": 0.7
                }
            
            self.adaptive_controller_A = AdaptiveSamplingController(
                max_iterations=max_iterations,
                **adaptive_config
            )
            self.adaptive_controller_B = AdaptiveSamplingController(
                max_iterations=max_iterations,
                **adaptive_config
            )
            # 初始化固定参数（用于自适应控制器更新前的默认值）
            self._fixed_gamma = 4.0
            self._fixed_p = 0.3
        else:
            self.adaptive_controller_A = None
            self.adaptive_controller_B = None
            # 无自适应控制器时使用固定参数
            self._fixed_gamma = 4.0
            self._fixed_p = 0.3
        
        # 提取环境参数（兼容多种 env 字段格式）
        self.dim = env.get('dimension', env.get('dim', 2))

        # 支持两种边界表示：'bounds' 或 'lower_bound'/'upper_bound'
        if 'bounds' in env:
            self.bounds = np.array(env['bounds'])
        elif 'lower_bound' in env and 'upper_bound' in env:
            lower = np.asarray(env['lower_bound']).flatten()
            upper = np.asarray(env['upper_bound']).flatten()
            # 构建 [xmin, xmax, ymin, ymax, ...]
            bounds_list = []
            for i in range(len(lower)):
                bounds_list.extend([float(lower[i]), float(upper[i])])
            self.bounds = np.array(bounds_list)
        else:
            raise KeyError("Environment must contain 'bounds' or ('lower_bound' and 'upper_bound')")

        self.start = np.asarray(env.get('start'))
        self.goal = np.asarray(env.get('goal'))

        # 支持 obstacles 为 numpy 数组 或 列表(dict) 的兼容转换
        obs = env.get('obstacles', None)
        if obs is None:
            self.obstacles = None
        else:
            # 列表形式，元素为 {'center': array, 'radius': r}
            if isinstance(obs, list) and len(obs) > 0 and isinstance(obs[0], dict):
                if self.dim == 2:
                    arr = np.array([[float(o['center'][0]), float(o['center'][1]), float(o['radius'])] for o in obs])
                else:
                    arr = np.array([[float(o['center'][0]), float(o['center'][1]), float(o['center'][2]), float(o['radius'])] for o in obs])
                self.obstacles = arr
            else:
                # 假设已经是 numpy array
                self.obstacles = np.asarray(obs)
        
        # 自动设置步长
        if step_size is None:
            if self.dim == 2:
                env_size = max(
                    self.bounds[1] - self.bounds[0],
                    self.bounds[3] - self.bounds[2]
                )
            else:
                env_size = max([
                    self.bounds[1] - self.bounds[0],
                    self.bounds[3] - self.bounds[2],
                    self.bounds[5] - self.bounds[4]
                ])
            
            base_step = env_size * self.STEP_SIZE_RATIO
            
            if self.dim == 2:
                self.step_size = np.clip(
                    base_step,
                    self.STEP_SIZE_MIN_2D,
                    self.STEP_SIZE_MAX_2D
                )
            else:
                self.step_size = np.clip(
                    base_step,
                    self.STEP_SIZE_MIN_3D,
                    self.STEP_SIZE_MAX_3D
                )
        else:
            self.step_size = step_size
        
        # 设置目标阈值
        self.goal_threshold = goal_threshold if goal_threshold else self.step_size
        
        if self.verbose:
            print(f"\n========== SC-RRT算法开始 ({self.dim}D) ==========")
            print(f"算法模式: {self.mode.upper()}")
            print(f"步长: {self.step_size:.2f}, 目标阈值: {self.goal_threshold:.2f}")
            print(f"最大迭代: {self.max_iterations}")
            print("=" * 50 + "\n")
    
    def plan(self) -> Tuple[Optional[np.ndarray], Dict, bool, Dict]:
        """
        执行路径规划
        
        Returns:
            path: 规划路径 (N, dim)
            tree: 树字典（包含treeA和treeB）
            success: 是否成功
            metrics: 性能指标字典
        """
        start_time = time.time()
        
        # 初始化双向树
        initial_capacity = min(1000, self.max_iterations // 10)
        
        # 树结构: [position(dim) | parent_idx | cost_G | cost_F_hat | num_children]
        treeA = np.zeros((initial_capacity, self.dim + 4))
        treeA[0, :self.dim] = self.start
        treeA[0, self.dim:self.dim+4] = [0, 0, np.inf, 0]
        sizeA = 1
        
        treeB = np.zeros((initial_capacity, self.dim + 4))
        treeB[0, :self.dim] = self.goal
        treeB[0, self.dim:self.dim+4] = [0, 0, np.inf, 0]
        sizeB = 1
        
        # 算法状态变量
        success = False
        path = None
        iterations = 0
        
        # 交汇点
        meet_point = (self.start + self.goal) / 2
        meet_point_old = meet_point.copy()
        
        # 双椭球体参数
        c_min_A = np.linalg.norm(meet_point - self.start)
        c_min_B = np.linalg.norm(self.goal - meet_point)
        c_best_A = np.inf
        c_best_B = np.inf
        
        # PID控制器状态
        prev_error_A = 0.0
        integral_error_A = 0.0
        prev_error_B = 0.0
        integral_error_B = 0.0
        
        # 性能指标追踪
        total_samples = 0
        valid_samples_A = 0
        valid_samples_B = 0
        L_best_shared = np.inf
        
        # ★★★ 有效采样比例(ESR)统计 ★★★
        # 统计"生成的采样点在椭球约束内的比例"，反映约束的引导有效性
        # ESR越高 → 约束越紧凑 → 采样越集中 → 引导性越强
        samples_in_ellipsoid_A = 0  # 树A：采样点真正在椭球内的数量
        samples_in_ellipsoid_B = 0  # 树B：采样点真正在椭球内的数量
        total_samples_A = 0  # 树A总采样次数
        total_samples_B = 0  # 树B总采样次数
        
        # 首次可行解追踪
        first_solution_found = False
        first_solution_iter = np.inf
        first_solution_time = np.inf
        first_path_length_raw = np.inf
        
        # 新增：详细指标跟踪
        tracking_errors = []  # 路径跟踪误差历史
        oscillation_count = 0  # 振荡次数
        last_direction = None  # 上次扩展方向
        ellipsoid_volumes = []  # 椭球体积历史
        failure_mode = 'none'  # 失败模式
        
        # ★★★ 自适应采样控制器历史记录 ★★★
        adaptive_sampling_history = {
            'iterations': [],
            'gamma_A': [],
            'gamma_B': [],
            'p_informed_A': [],
            'p_informed_B': [],
            'c_best_A': [],
            'c_best_B': [],
            'phase_A': [],
            'phase_B': []
        }
        
        # 收敛历史
        convergence_history = {
            'iterations': [],
            'best_path_length': [],
            'tree_sizes': [],
            'elapsed_time': []
        }
        history_record_interval = max(5, self.max_iterations // 200)
        
        # 提前终止参数
        MAX_EXTRA_ITERS = max(200, int(self.max_iterations * 0.05))
        extra_optimization_iters = 0
        
        # 主循环
        while iterations < self.max_iterations:
            iterations += 1
            
            # 记录收敛历史
            if iterations % history_record_interval == 0:
                convergence_history['iterations'].append(iterations)
                convergence_history['best_path_length'].append(L_best_shared)
                convergence_history['tree_sizes'].append(sizeA + sizeB)
                convergence_history['elapsed_time'].append(time.time() - start_time)
                
                # 记录椭球体积（用于分析约束收缩速率）
                if sizeA > 1 and sizeB > 1:
                    volume = self._calculate_ellipsoid_volume(c_best_A, c_best_B, meet_point)
                    ellipsoid_volumes.append(volume)
            
            # 提前终止检查
            if success and iterations > first_solution_iter:
                extra_optimization_iters = iterations - first_solution_iter
                if extra_optimization_iters >= MAX_EXTRA_ITERS:
                    if self.verbose:
                        print(f"✓ 已找到解并优化{extra_optimization_iters}次迭代，提前终止")
                    break
            
            # 动态扩展树数组
            if sizeA >= len(treeA):
                treeA = np.vstack([treeA, np.zeros((initial_capacity, self.dim + 4))])
            if sizeB >= len(treeB):
                treeB = np.vstack([treeB, np.zeros((initial_capacity, self.dim + 4))])
            
            # ★★★ Pareto节点选择策略：周期性选择更优节点作为新的采样参考点 ★★★
            self.pareto_counter += 1
            if self.use_pareto and self.pareto_counter >= self.pareto_interval and sizeA > 10 and sizeB > 10:
                # 为树A选择Pareto最优节点，更新meet_point的A侧参考
                ref_A_idx = 0  # 当前从start开始
                new_ref_A_idx, new_ref_A = self._choose_node_pareto(
                    treeA[:sizeA], meet_point, ref_A_idx, self.pareto_prob
                )
                
                # 为树B选择Pareto最优节点，更新meet_point的B侧参考
                ref_B_idx = 0  # 当前从goal开始
                new_ref_B_idx, new_ref_B = self._choose_node_pareto(
                    treeB[:sizeB], meet_point, ref_B_idx, self.pareto_prob
                )
                
                # 如果选择了不同的节点，更新meet_point为两个新参考点的中点
                if new_ref_A_idx != ref_A_idx or new_ref_B_idx != ref_B_idx:
                    # 计算新的meet_point（两个Pareto最优节点的中点）
                    meet_point_pareto = (new_ref_A + new_ref_B) / 2
                    
                    # 平滑更新meet_point（避免突变）
                    meet_point = 0.7 * meet_point + 0.3 * meet_point_pareto
                    meet_point_old = meet_point.copy()
                    
                    # 记录Pareto选择历史
                    self.pareto_selections.append({
                        'iteration': iterations,
                        'ref_A_idx': new_ref_A_idx,
                        'ref_B_idx': new_ref_B_idx,
                        'new_meet_point': meet_point.copy(),
                        'distance': np.linalg.norm(new_ref_A - new_ref_B)
                    })
                    
                    if self.verbose:
                        print(f"  ★ Pareto选择: A节点{new_ref_A_idx}, B节点{new_ref_B_idx}, 新meet_point距离={np.linalg.norm(new_ref_A - new_ref_B):.2f}")
                
                self.pareto_counter = 0  # 重置计数器
            
            # 交汇点更新
            adaptive_interval = max(20, min(100, int(50 * (1 - iterations / self.max_iterations))))
            if iterations % adaptive_interval == 0 and sizeA > 1 and sizeB > 1:
                meet_point_new = self._calculate_meet_point(
                    treeA[:sizeA], treeB[:sizeB], meet_point
                )
                meet_point = self.smoothing_factor * meet_point_old + \
                            (1 - self.smoothing_factor) * meet_point_new
                meet_point_old = meet_point.copy()
            
            # ★★★ 新增：自适应采样控制器更新（替代PID控制器）★★★
            if self.use_adaptive_controller:
                # 1. 更新自适应控制器，获取gamma和p_informed
                # 注意：AdaptiveSamplingController内部会计算采样效率
                gamma_A, p_informed_A, _ = self.adaptive_controller_A.update(
                    iteration=iterations,
                    has_solution=success,
                    tree_size_a=sizeA,
                    tree_size_b=sizeB,
                    recent_valid_samples=samples_in_ellipsoid_A,
                    recent_total_samples=total_samples_A
                )
                gamma_B, p_informed_B, _ = self.adaptive_controller_B.update(
                    iteration=iterations,
                    has_solution=success,
                    tree_size_a=sizeB,
                    tree_size_b=sizeA,
                    recent_valid_samples=samples_in_ellipsoid_B,
                    recent_total_samples=total_samples_B
                )
                
                # 3. 计算焦距
                c_min_A = np.linalg.norm(meet_point - self.start)
                c_min_B = np.linalg.norm(self.goal - meet_point)
                
                # 4. 使用gamma调整椭球大小
                c_best_A = c_min_A * gamma_A
                c_best_B = c_min_B * gamma_B
                
                # 5. 保存到变量供采样使用
                self._gamma_A = gamma_A
                self._gamma_B = gamma_B
                self._p_informed_A = p_informed_A
                self._p_informed_B = p_informed_B
                
                # 6. 记录历史
                if iterations % 20 == 0:
                    adaptive_sampling_history['iterations'].append(iterations)
                    adaptive_sampling_history['gamma_A'].append(gamma_A)
                    adaptive_sampling_history['gamma_B'].append(gamma_B)
                    adaptive_sampling_history['p_informed_A'].append(p_informed_A)
                    adaptive_sampling_history['p_informed_B'].append(p_informed_B)
                    adaptive_sampling_history['c_best_A'].append(c_best_A)
                    adaptive_sampling_history['c_best_B'].append(c_best_B)
                    adaptive_sampling_history['phase_A'].append(str(self.adaptive_controller_A.current_phase.value))
                    adaptive_sampling_history['phase_B'].append(str(self.adaptive_controller_B.current_phase.value))
                
                # 4. 计算焦距
                c_min_A = np.linalg.norm(meet_point - self.start)
                c_min_B = np.linalg.norm(self.goal - meet_point)
                
                # 5. 使用gamma调整椭球大小
                c_best_A = c_min_A * gamma_A
                c_best_B = c_min_B * gamma_B
                
                # 6. 记录自适应采样历史（用于可视化）
                if iterations % 20 == 0:
                    adaptive_sampling_history['iterations'].append(iterations)
                    adaptive_sampling_history['gamma_A'].append(gamma_A)
                    adaptive_sampling_history['gamma_B'].append(gamma_B)
                    adaptive_sampling_history['p_informed_A'].append(p_informed_A)
                    adaptive_sampling_history['p_informed_B'].append(p_informed_B)
                    adaptive_sampling_history['c_best_A'].append(c_best_A)
                    adaptive_sampling_history['c_best_B'].append(c_best_B)
                    adaptive_sampling_history['phase_A'].append(str(self.adaptive_controller_A.current_phase.value))
                    adaptive_sampling_history['phase_B'].append(str(self.adaptive_controller_B.current_phase.value))
                
                # 保存到变量供采样使用
                self._gamma_A = gamma_A
                self._gamma_B = gamma_B
                self._p_informed_A = p_informed_A
                self._p_informed_B = p_informed_B
                
            else:
                # 无自适应控制器：使用固定参数
                c_min_A = np.linalg.norm(meet_point - self.start)
                c_min_B = np.linalg.norm(self.goal - meet_point)
                
                c_best_A = c_min_A * self._fixed_gamma
                c_best_B = c_min_B * self._fixed_gamma
                
                self._gamma_A = self._fixed_gamma
                self._gamma_B = self._fixed_gamma
                self._p_informed_A = self._fixed_p
                self._p_informed_B = self._fixed_p
            
            # 打印进度
            if self.verbose and iterations % max(50, self.max_iterations // 20) == 0:
                print(f"迭代{iterations}/{self.max_iterations}: "
                      f"树A={sizeA}节点, 树B={sizeB}节点, "
                      f"最优={L_best_shared:.2f}")
            
            # 周期性全局连接检查
            global_check_interval = max(800, self.max_iterations // 6)
            if iterations % global_check_interval == 0 and sizeA > 10 and sizeB > 10:
                best_path, best_cost = self._try_connect_trees(
                    treeA[:sizeA], treeB[:sizeB]
                )
                if best_path is not None and best_cost < L_best_shared:
                    path = best_path
                    L_best_shared = best_cost
                    
                    if not first_solution_found:
                        first_solution_found = True
                        first_solution_iter = iterations
                        first_solution_time = time.time() - start_time
                        first_path_length_raw = best_cost
                    
                    success = True
                    if self.verbose:
                        print(f"✓ 发现更优连接! 代价: {best_cost:.2f}")
            
            # 扩展树A（从起点向交汇点）
            total_samples += 1
            total_samples_A += 1  # ★ ESR统计：树A采样计数
            
            # ★★★ 关键修复：所有模式都使用椭球约束，但PID动态调节大小 ★★★
            in_ellipsoid_A = False  # 默认不是椭球引导采样
            if self.mode == 'no_pid':
                # No_PID模式：使用椭球约束但不动态调节（公平对比基准）
                # 注意：c_best_A已经在_calculate_ellipsoid_params中计算为固定值
                x_rand_A, _, _, in_ellipsoid_A = self._sample_with_pid(
                    meet_point, treeA[:sizeA], c_best_A,
                    prev_error_A, integral_error_A, 'A'
                )
            else:
                # PID控制采样（使用动态调节的椭球约束）
                x_rand_A, prev_error_A, integral_error_A, in_ellipsoid_A = self._sample_with_pid(
                    meet_point, treeA[:sizeA], c_best_A,
                    prev_error_A, integral_error_A, 'A'
                )
            
            # ★★★ ESR统计：记录采样点是否在椭球约束内 ★★★
            if in_ellipsoid_A:
                samples_in_ellipsoid_A += 1  # 采样点在椭球内
            
            if x_rand_A is not None:
                nearest_idx = find_nearest_node(treeA[:sizeA], x_rand_A, self.dim)
                x_near = treeA[nearest_idx, :self.dim]
                x_new = steer_point(x_near, x_rand_A, self.step_size)
                
                if is_collision_free(x_near, x_new, self.obstacles, self.dim):
                    # 添加新节点
                    cost_new = treeA[nearest_idx, self.dim+1] + np.linalg.norm(x_new - x_near)
                    treeA[sizeA, :self.dim] = x_new
                    treeA[sizeA, self.dim:self.dim+4] = [nearest_idx, cost_new, np.inf, 0]
                    sizeA += 1
                    valid_samples_A += 1
                    
                    # 跟踪路径误差（到交汇点的距离）
                    tracking_error = np.linalg.norm(x_new - meet_point)
                    tracking_errors.append(tracking_error)
                    
                    # 检测振荡（方向改变）
                    current_direction = x_new - x_near
                    if last_direction is not None:
                        cos_angle = np.dot(current_direction, last_direction) / \
                                   (np.linalg.norm(current_direction) * np.linalg.norm(last_direction) + 1e-10)
                        if cos_angle < -0.5:  # 方向反转超过120度
                            oscillation_count += 1
                    last_direction = current_direction
                    
                    # 检查是否到达交汇点附近
                    if np.linalg.norm(x_new - meet_point) < self.goal_threshold:
                        # 尝试连接到树B
                        nearest_B_idx = find_nearest_node(treeB[:sizeB], x_new, self.dim)
                        x_near_B = treeB[nearest_B_idx, :self.dim]
                        
                        if is_collision_free(x_new, x_near_B, self.obstacles, self.dim):
                            # 构建完整路径
                            total_cost = cost_new + treeB[nearest_B_idx, self.dim+1] + \
                                       np.linalg.norm(x_new - x_near_B)
                            
                            if total_cost < L_best_shared:
                                path = self._extract_path(
                                    treeA[:sizeA], treeB[:sizeB],
                                    sizeA - 1, nearest_B_idx
                                )
                                L_best_shared = total_cost
                                
                                if not first_solution_found:
                                    first_solution_found = True
                                    first_solution_iter = iterations
                                    first_solution_time = time.time() - start_time
                                    first_path_length_raw = total_cost
                                
                                success = True
            
            # 扩展树B（从终点向交汇点）
            total_samples += 1
            total_samples_B += 1  # ★ ESR统计：树B采样计数
            
            # ★★★ 关键修复：所有模式都使用椭球约束 ★★★
            in_ellipsoid_B = False  # 默认不是椭球引导采样
            if self.mode == 'no_pid':
                # No_PID模式：使用固定椭球约束
                x_rand_B, _, _, in_ellipsoid_B = self._sample_with_pid(
                    meet_point, treeB[:sizeB], c_best_B,
                    prev_error_B, integral_error_B, 'B'
                )
            else:
                # PID控制采样：使用动态调节的椭球约束
                x_rand_B, prev_error_B, integral_error_B, in_ellipsoid_B = self._sample_with_pid(
                    meet_point, treeB[:sizeB], c_best_B,
                    prev_error_B, integral_error_B, 'B'
                )
            
            # ★★★ ESR统计：记录采样点是否在椭球约束内 ★★★
            if in_ellipsoid_B:
                samples_in_ellipsoid_B += 1  # 采样点在椭球内
            
            if x_rand_B is not None:
                nearest_idx = find_nearest_node(treeB[:sizeB], x_rand_B, self.dim)
                x_near = treeB[nearest_idx, :self.dim]
                x_new = steer_point(x_near, x_rand_B, self.step_size)
                
                if is_collision_free(x_near, x_new, self.obstacles, self.dim):
                    cost_new = treeB[nearest_idx, self.dim+1] + np.linalg.norm(x_new - x_near)
                    treeB[sizeB, :self.dim] = x_new
                    treeB[sizeB, self.dim:self.dim+4] = [nearest_idx, cost_new, np.inf, 0]
                    sizeB += 1
                    valid_samples_B += 1
        
        # 计算最终指标
        planning_time = time.time() - start_time
        
        # 判断失败模式
        if not success:
            if sizeA + sizeB < 100:
                failure_mode = 'early_stop'  # 早期卡死
            elif len(tracking_errors) > 0 and np.mean(tracking_errors[-50:]) > np.mean(tracking_errors[:50]):
                failure_mode = 'divergence'  # 路径发散
            elif oscillation_count > iterations * 0.1:
                failure_mode = 'oscillation'  # 振荡过多
            else:
                failure_mode = 'timeout'  # 超时
        
        metrics = {
            'iterations': iterations,
            'tree_nodes': sizeA + sizeB,
            'path_length': L_best_shared if success else np.inf,
            'planning_time': planning_time,
            'success_rate': 1.0 if success else 0.0,
            'first_solution_iter': first_solution_iter if first_solution_found else np.inf,
            'first_path_length': first_path_length_raw if first_solution_found else np.inf,
            'convergence_time': first_solution_time if first_solution_found else np.inf,
            'convergence_history': convergence_history,
            # 新增详细指标
            'avg_tracking_error': np.mean(tracking_errors) if len(tracking_errors) > 0 else np.inf,
            'oscillation_count': oscillation_count,
            'oscillation_rate': oscillation_count / max(1, iterations),
            'failure_mode': failure_mode,
            'ellipsoid_volumes': ellipsoid_volumes,
            'final_ellipsoid_volume': ellipsoid_volumes[-1] if len(ellipsoid_volumes) > 0 else np.inf,
            # ★★★ 有效采样比例(ESR)核心指标 ★★★
            # ESR = 生成的采样点在椭球约束内的比例（反映约束的引导有效性）
            # ESR越高 → 约束越紧 → 采样越集中 → SC-RRT的优势越明显
            'effective_sampling_ratio': (samples_in_ellipsoid_A + samples_in_ellipsoid_B) / max(1, total_samples_A + total_samples_B),
            'samples_in_ellipsoid': samples_in_ellipsoid_A + samples_in_ellipsoid_B,
            'total_samples_attempted': total_samples_A + total_samples_B,
            'esr_tree_A': samples_in_ellipsoid_A / max(1, total_samples_A),
            'esr_tree_B': samples_in_ellipsoid_B / max(1, total_samples_B),
            # ★★★ 自适应采样控制器历史数据 ★★★
            'adaptive_sampling_history': adaptive_sampling_history if self.use_adaptive_controller else None
        }
        
        if success and path is not None:
            metrics['smoothness'] = calculate_path_smoothness(path)
            metrics['clearance'] = self._calculate_clearance(path)
        else:
            metrics['smoothness'] = np.inf
            metrics['clearance'] = 0.0
        
        if self.verbose:
            print(f"\n规划完成: {'成功' if success else '失败'}")
            print(f"总时间: {planning_time:.2f}s, 迭代: {iterations}")
            if success:
                print(f"路径长度: {metrics['path_length']:.2f}")
                print(f"首次解: 第{first_solution_iter}次迭代 ({first_solution_time:.2f}s)")
        
        tree = {'treeA': treeA[:sizeA], 'treeB': treeB[:sizeB]}
        
        return path, tree, success, metrics
    
    def _sample_with_pid(
        self, target: np.ndarray, tree: np.ndarray,
        c_best: float, prev_error: float, integral_error: float,
        tree_name: str
    ) -> Tuple[Optional[np.ndarray], float, float, bool]:
        """
        采样策略（支持自适应控制器）
        
        Returns:
            sample: 采样点
            current_error: 当前误差（保持接口兼容）
            integral_error: 积分误差（保持接口兼容）
            in_ellipsoid: 采样点是否真正在椭球约束内（用于ESR统计）
        """
        # 获取控制参数
        if tree_name == 'A':
            focus1, focus2 = self.start, target
            gamma = getattr(self, '_gamma_A', self._fixed_gamma)
            p_informed = getattr(self, '_p_informed_A', self._fixed_p)
        else:
            focus1, focus2 = target, self.goal
            gamma = getattr(self, '_gamma_B', self._fixed_gamma)
            p_informed = getattr(self, '_p_informed_B', self._fixed_p)
        
        # 计算当前误差（保持接口兼容）
        if len(tree) > 1:
            distances = np.linalg.norm(tree[:, :self.dim] - target, axis=1)
            min_dist = np.min(distances)
            env_diagonal = np.linalg.norm(self.goal - self.start)
            current_error = min_dist / (env_diagonal + 1e-10)
        else:
            current_error = 1.0
        
        # 采样策略：基于p_informed概率选择椭球采样或随机采样
        rand_val = np.random.rand()
        sample = None
        
        if rand_val < p_informed:
            # ★★★ 优化：椭球引导采样（带边界约束） ★★★
            # 根据椭球大小动态调整尝试次数
            if gamma > 4.0:
                max_attempts = 15  # 大椭球，多尝试
            elif gamma > 3.0:
                max_attempts = 10
            else:
                max_attempts = 5   # 小椭球，少尝试
            
            sample = sample_in_ellipsoid(focus1, focus2, c_best, self.dim, 
                                        bounds=self.bounds, max_attempts=max_attempts)
            if sample is not None:
                # 成功生成椭球内采样点
                return sample, current_error, integral_error, True
            
            # ★★★ 优化：椭球采样失败后的智能回退策略 ★★★
            # 不直接跳到完全随机，而是生成"偏向椭球中心"的采样点
            center = (focus1 + focus2) / 2
            random_point = sample_point(self.bounds, self.dim)
            
            # 向椭球中心偏移50%
            direction = center - random_point
            sample = random_point + 0.5 * direction
            
            # 裁剪到边界内
            for i in range(self.dim):
                sample[i] = np.clip(sample[i], self.bounds[2*i], self.bounds[2*i+1])
        else:
            # 随机探索采样
            sample = sample_point(self.bounds, self.dim)
        
        # ★★★ 检查最终采样点是否在椭球约束内（用于ESR统计） ★★★
        in_ellipsoid = self._is_in_ellipsoid(sample, focus1, focus2, c_best)
        
        return sample, current_error, integral_error, in_ellipsoid
    
    def _is_in_ellipsoid(self, point: np.ndarray, focus1: np.ndarray, 
                         focus2: np.ndarray, c_max: float) -> bool:
        """判断点是否在椭球约束内（用于ESR统计）
        
        椭球定义：到两焦点距离之和 <= c_max
        """
        dist_sum = np.linalg.norm(point - focus1) + np.linalg.norm(point - focus2)
        return dist_sum <= c_max
    
    def _calculate_meet_point(
        self, treeA: np.ndarray, treeB: np.ndarray, current_meet: np.ndarray
    ) -> np.ndarray:
        """计算交汇点 - 改进版：使用加权质心 + 最近节点对"""
        # 策略1：加权质心（靠近目标的节点权重更高）
        dist_to_goal_A = np.linalg.norm(treeA[:, :self.dim] - self.goal, axis=1)
        dist_to_start_B = np.linalg.norm(treeB[:, :self.dim] - self.start, axis=1)
        
        # 权重：距离越近权重越高（使用softmax归一化）
        weights_A = np.exp(-dist_to_goal_A / np.mean(dist_to_goal_A))
        weights_A /= weights_A.sum()
        weights_B = np.exp(-dist_to_start_B / np.mean(dist_to_start_B))
        weights_B /= weights_B.sum()
        
        weighted_centroid_A = np.sum(treeA[:, :self.dim] * weights_A[:, np.newaxis], axis=0)
        weighted_centroid_B = np.sum(treeB[:, :self.dim] * weights_B[:, np.newaxis], axis=0)
        
        # 策略2：找到两树中距离最近的节点对
        min_dist = np.inf
        best_pair_midpoint = current_meet
        
        # 采样策略：从每棵树中选择前20%最靠近对方的节点
        sample_size_A = max(5, len(treeA) // 5)
        sample_size_B = max(5, len(treeB) // 5)
        
        # 树A中最靠近树B的节点
        dist_A_to_B_centroid = np.linalg.norm(treeA[:, :self.dim] - weighted_centroid_B, axis=1)
        top_A_indices = np.argsort(dist_A_to_B_centroid)[:sample_size_A]
        
        # 树B中最靠近树A的节点
        dist_B_to_A_centroid = np.linalg.norm(treeB[:, :self.dim] - weighted_centroid_A, axis=1)
        top_B_indices = np.argsort(dist_B_to_A_centroid)[:sample_size_B]
        
        # 在这些候选节点中找最近对
        for i in top_A_indices:
            for j in top_B_indices:
                dist = np.linalg.norm(treeA[i, :self.dim] - treeB[j, :self.dim])
                if dist < min_dist:
                    min_dist = dist
                    best_pair_midpoint = (treeA[i, :self.dim] + treeB[j, :self.dim]) / 2
        
        # 策略3：融合三种估计（加权平均）
        # - 加权质心中点：40%权重（考虑全局分布）
        # - 最近节点对中点：40%权重（考虑局部连接）
        # - 当前meet_point：20%权重（保持稳定性）
        centroid_midpoint = (weighted_centroid_A + weighted_centroid_B) / 2
        
        new_meet = (0.4 * centroid_midpoint + 
                   0.4 * best_pair_midpoint + 
                   0.2 * current_meet)
        
        return new_meet
    
    def _choose_node_pareto(
        self, tree: np.ndarray, target: np.ndarray, 
        current_ref_idx: int, pareto_prob: float
    ) -> Tuple[int, np.ndarray]:
        """
        ★★★ Pareto优势选择策略 ★★★
        从树中选择Pareto前沿上的节点作为新的采样参考点
        
        目标：选择同时优化两个目标的节点：
        1. 到目标点的距离（距离越小越好）
        2. 路径代价（cost越小越好）
        
        Args:
            tree: 节点树 [position | parent_idx | cost_G | cost_F_hat | num_children]
            target: 目标点（通常是meet_point）
            current_ref_idx: 当前参考节点索引
            pareto_prob: Pareto选择概率（用于随机性）
            
        Returns:
            new_ref_idx: 新的参考节点索引
            new_ref_point: 新的参考节点位置
        """
        if len(tree) < 5:  # 节点太少，不进行选择
            return current_ref_idx, tree[current_ref_idx, :self.dim]
        
        # 1. 计算所有节点的两个目标值
        positions = tree[:, :self.dim]
        costs = tree[:, self.dim+1]  # cost_G
        
        # 目标1：到目标点的距离
        distances_to_target = np.linalg.norm(positions - target, axis=1)
        
        # 目标2：路径代价
        path_costs = costs
        
        # 2. 归一化目标值（便于比较）
        dist_min, dist_max = distances_to_target.min(), distances_to_target.max()
        cost_min, cost_max = path_costs.min(), path_costs.max()
        
        if dist_max > dist_min:
            norm_distances = (distances_to_target - dist_min) / (dist_max - dist_min)
        else:
            norm_distances = np.zeros_like(distances_to_target)
        
        if cost_max > cost_min:
            norm_costs = (path_costs - cost_min) / (cost_max - cost_min)
        else:
            norm_costs = np.zeros_like(path_costs)
        
        # 3. 找出Pareto前沿节点（非支配解）
        pareto_front = []
        for i in range(len(tree)):
            is_dominated = False
            for j in range(len(tree)):
                if i == j:
                    continue
                # 如果j支配i（j在两个目标上都不差于i，且至少一个更好）
                if (distances_to_target[j] <= distances_to_target[i] and 
                    path_costs[j] <= path_costs[i] and
                    (distances_to_target[j] < distances_to_target[i] or 
                     path_costs[j] < path_costs[i])):
                    is_dominated = True
                    break
            
            if not is_dominated:
                pareto_front.append(i)
        
        if len(pareto_front) == 0:
            pareto_front = [0]  # 回退到根节点
        
        # 4. 从Pareto前沿中选择节点（基于概率和综合得分）
        if np.random.rand() < pareto_prob:
            # 高概率选择：根据综合得分（平衡距离和代价）
            # 综合得分 = 0.6 * 归一化距离 + 0.4 * 归一化代价（距离更重要）
            scores = 0.6 * norm_distances[pareto_front] + 0.4 * norm_costs[pareto_front]
            best_idx_in_front = np.argmin(scores)
            selected_idx = pareto_front[best_idx_in_front]
        else:
            # 低概率随机选择：增加探索性
            selected_idx = np.random.choice(pareto_front)
        
        # 5. 碰撞检测：确保从当前参考点到新参考点的路径无碰撞
        current_ref = tree[current_ref_idx, :self.dim]
        new_ref = tree[selected_idx, :self.dim]
        
        if not is_collision_free(current_ref, new_ref, self.obstacles, self.dim):
            # 如果有碰撞，保持当前参考点
            return current_ref_idx, current_ref
        
        return selected_idx, new_ref
    
    def _calculate_ellipsoid_params(
        self, treeA: np.ndarray, treeB: np.ndarray, meet_point: np.ndarray,
        prev_error_A: float, integral_error_A: float,
        prev_error_B: float, integral_error_B: float
    ) -> Tuple[float, float, float, float, float, float, float, float]:
        """计算双椭球体参数 - 核心修复：使用真正的PID参数控制椭球长轴
        
        Returns:
            c_best_A, c_best_B, c_min_A, c_min_B, 
            current_error_A, integral_error_A, current_error_B, integral_error_B
        """
        # 焦距（直线距离）
        c_min_A = np.linalg.norm(meet_point - self.start)
        c_min_B = np.linalg.norm(self.goal - meet_point)
        
        # ★★★ 核心修复：根据模式选择椭球控制策略 ★★★
        if self.mode == 'no_pid':
            # No_PID模式：使用固定椭球约束（公平对比的基准）
            c_best_A = c_min_A * 5.0  # 固定5.0倍（考虑障碍物绕行）
            c_best_B = c_min_B * 5.0
            current_error_A = 0.0
            current_error_B = 0.0
        else:
            # ★★★ PID模式：使用真正的Kp/Ki/Kd参数动态调节椭球大小 ★★★
            # 计算当前误差（归一化相对误差）
            if len(treeA) > 1:
                distances_A = np.linalg.norm(treeA[:, :self.dim] - meet_point, axis=1)
                actual_cost_A = np.min(distances_A)  # F(x)
                estimated_cost_A = c_min_A  # F_hat(x)
                current_error_A = abs(actual_cost_A - estimated_cost_A) / (estimated_cost_A + 1.0)
            else:
                current_error_A = 0.5
            
            if len(treeB) > 1:
                distances_B = np.linalg.norm(treeB[:, :self.dim] - meet_point, axis=1)
                actual_cost_B = np.min(distances_B)
                estimated_cost_B = c_min_B
                current_error_B = abs(actual_cost_B - estimated_cost_B) / (estimated_cost_B + 1.0)
            else:
                current_error_B = 0.5
            
            # ★★★ 真正的PID控制器计算 ★★★
            # P项：比例控制
            P_term_A = self.custom_Kp * current_error_A
            P_term_B = self.custom_Kp * current_error_B
            
            # I项：积分控制（累积误差→消除稳态误差）
            integral_error_A += current_error_A
            integral_error_B += current_error_B
            I_term_A = self.custom_Ki * integral_error_A
            I_term_B = self.custom_Ki * integral_error_B
            
            # D项：微分控制（误差变化率→阻尼振荡）
            error_derivative_A = current_error_A - prev_error_A
            error_derivative_B = current_error_B - prev_error_B
            D_term_A = self.custom_Kd * error_derivative_A
            D_term_B = self.custom_Kd * error_derivative_B
            
            # PID总输出
            pid_output_A = P_term_A + I_term_A + D_term_A
            pid_output_B = P_term_B + I_term_B + D_term_B
            
            # ★★★ 关键修复：椭球收缩策略（收敛控制）★★★
            # 正确的物理意义：
            # - 误差大（树离目标远）→ 收紧椭球 → 集中搜索朝目标 → 提高成功率
            # - 误差小（树接近目标）→ 放松椭球 → 允许探索更优路径 → 提高路径质量
            #
            # 实现：基准倍数5.0（考虑障碍物绕行），PID在此基础上收缩
            # - pid_output大（误差大）→ factor = 5.0 - 2.0 = 3.0 → 椭球收紧
            # - pid_output小（误差小）→ factor = 5.0 - 0.0 = 5.0 → 椭球放松
            pid_factor_A = 5.0 - np.clip(pid_output_A, 0.0, 2.0)  # [3.0, 5.0]
            pid_factor_B = 5.0 - np.clip(pid_output_B, 0.0, 2.0)
            
            # 最终椭球长轴 = 焦距 × PID动态因子
            c_best_A = c_min_A * pid_factor_A
            c_best_B = c_min_B * pid_factor_B
            
            # 限制调节范围，防止过小或过大（与No_PID基准保持一致）
            c_best_A = np.clip(c_best_A, c_min_A * 3.0, c_min_A * 6.0)
            c_best_B = np.clip(c_best_B, c_min_B * 3.0, c_min_B * 6.0)
        
        return c_best_A, c_best_B, c_min_A, c_min_B, current_error_A, integral_error_A, current_error_B, integral_error_B
    
    def _try_connect_trees(
        self, treeA: np.ndarray, treeB: np.ndarray
    ) -> Tuple[Optional[np.ndarray], float]:
        """尝试连接两棵树"""
        best_path = None
        best_cost = np.inf
        max_check = min(20, max(8, int(np.sqrt(min(len(treeA), len(treeB))) * 0.5)))
        
        for _ in range(max_check):
            idx_A = np.random.randint(0, len(treeA))
            idx_B = np.random.randint(0, len(treeB))
            
            node_A = treeA[idx_A, :self.dim]
            node_B = treeB[idx_B, :self.dim]
            
            if is_collision_free(node_A, node_B, self.obstacles, self.dim):
                cost = treeA[idx_A, self.dim+1] + treeB[idx_B, self.dim+1] + \
                       np.linalg.norm(node_A - node_B)
                
                if cost < best_cost:
                    best_cost = cost
                    best_path = self._extract_path(treeA, treeB, idx_A, idx_B)
        
        return best_path, best_cost
    
    def _extract_path(
        self, treeA: np.ndarray, treeB: np.ndarray,
        idx_A: int, idx_B: int
    ) -> np.ndarray:
        """从两棵树提取路径"""
        # 从树A回溯到起点
        path_A = []
        current = idx_A
        while current != 0:
            path_A.append(treeA[current, :self.dim])
            current = int(treeA[current, self.dim])
        path_A.append(self.start)
        path_A.reverse()
        
        # 从树B回溯到终点
        path_B = []
        current = idx_B
        while current != 0:
            path_B.append(treeB[current, :self.dim])
            current = int(treeB[current, self.dim])
        path_B.append(self.goal)
        
        # 合并路径
        path = path_A + path_B
        return np.array(path)
    
    def _calculate_clearance(self, path: np.ndarray) -> float:
        """计算平均障碍物间隙"""
        if len(self.obstacles) == 0:
            return np.inf
        
        clearances = []
        for point in path:
            if self.dim == 2:
                distances = np.linalg.norm(self.obstacles[:, :2] - point, axis=1)
                min_clearance = np.min(distances - self.obstacles[:, 2])
            else:
                distances = np.linalg.norm(self.obstacles[:, :3] - point, axis=1)
                min_clearance = np.min(distances - self.obstacles[:, 3])
            clearances.append(max(0, min_clearance))
        
        return np.mean(clearances)
    
    def _calculate_ellipsoid_volume(self, c_best_A: float, c_best_B: float, meet_point: np.ndarray) -> float:
        """计算椭球体积（用于约束收缩分析）"""
        # 简化体积计算（椭球长半轴 = c_best）
        if self.dim == 2:
            # 2D椭圆面积：π * a * b
            return np.pi * c_best_A * c_best_B
        else:
            # 3D椭球体积：(4/3) * π * a * b * c
            return (4/3) * np.pi * c_best_A * c_best_B * (c_best_A + c_best_B) / 2
    
    def _pure_random_sample(self, tree_name: str, other_tree: np.ndarray) -> np.ndarray:
        """纯随机采样（无PID模式专用）：90%完全随机 + 10%朝对方树"""
        if np.random.rand() < 0.1 and len(other_tree) > 0:
            # 10%概率朝对方树采样（基本RRT连接策略）
            target_idx = np.random.randint(0, len(other_tree))
            target_point = other_tree[target_idx, :self.dim]
            # 添加大噪声避免过于定向
            noise = np.random.randn(self.dim) * np.linalg.norm(self.goal - self.start) * 0.3
            sample = target_point + noise
            # 边界裁剪
            if self.dim == 2:
                sample = np.clip(sample, [self.bounds[0], self.bounds[2]], [self.bounds[1], self.bounds[3]])
            else:
                sample = np.clip(sample, [self.bounds[0], self.bounds[2], self.bounds[4]], 
                                [self.bounds[1], self.bounds[3], self.bounds[5]])
            return sample
        else:
            # 90%完全随机采样
            return sample_point(self.bounds, self.dim)
    
    def _calculate_c_best_from_tree(
        self, tree: np.ndarray, target_point: np.ndarray, base_point: np.ndarray
    ) -> float:
        """
        从树中计算到目标点的最优代价（用于PID采样控制器）
        
        参考MATLAB版本的calculateDualEllipsoidParams.m
        遍历树节点，找到最小的 F(i) = G(i) + H(i, target)
        
        Args:
            tree: 树结构 [N, dim+4]
            target_point: 目标点（交汇点）
            base_point: 基准点（起点或终点）
            
        Returns:
            c_best: 最优代价
        """
        if len(tree) < 1:
            return np.inf
        
        c_best = np.inf
        for i in range(len(tree)):
            # G(i): 基准点到节点i的实际代价
            G_i = tree[i, self.dim + 1]
            
            # H(i, target): 节点i到目标点的启发式代价（欧氏距离）
            H_i = np.linalg.norm(tree[i, :self.dim] - target_point)
            
            # F(i) = G(i) + H(i)
            F_i = G_i + H_i
            
            if F_i < c_best:
                c_best = F_i
        
        # 确保不返回Inf（至少返回焦距）
        if np.isinf(c_best):
            c_best = np.linalg.norm(target_point - base_point) * 5.0
        
        return c_best

