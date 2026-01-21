"""
SC-RRT Basic PID 算法实现
Self-Constrained RRT with PID Control (Basic Version)

核心特点:
1. 双向RRT with 非对称双椭球约束采样
2. 动态交汇点转移机制
3. PID自适应权重控制
4. 支持自定义PID参数进行批量调优
"""

import numpy as np
import time
from typing import Dict, Optional, Tuple, List
from .geometry import (
    is_collision_free, steer_point, sample_point,
    sample_in_ellipsoid, calculate_path_length,
    calculate_path_smoothness, find_nearest_node
)
from .pid_controller import PIDWeightController


class SCRRTBasicPID:
    """SC-RRT Basic PID 算法类"""
    
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
        mode: str = 'custom_pid',
        Kp: float = 0.25,
        Ki: float = 0.04,
        Kd: float = 0.10,
        step_size: Optional[float] = None,
        goal_threshold: Optional[float] = None,
        update_interval: int = 50,
        smoothing_factor: float = 0.7,
        ellipsoid_buffer: float = 1.2,
        use_pareto: bool = True,
        verbose: bool = True
    ):
        """
        初始化SC-RRT Basic PID
        
        Args:
            env: 环境字典
            max_iterations: 最大迭代次数
            mode: 控制模式 ('basic'|'pid'|'adaptive'|'custom_pid'|'no_pid')
            Kp: 比例增益
            Ki: 积分增益
            Kd: 微分增益
            step_size: 步长（None则自动设置）
            goal_threshold: 目标阈值（None则自动设置）
            update_interval: 交汇点更新间隔
            smoothing_factor: 交汇点平滑因子
            ellipsoid_buffer: 椭球体缓冲系数
            use_pareto: 是否使用Pareto前沿
            verbose: 是否打印详细信息
        """
        self.env = env
        self.max_iterations = max_iterations
        self.mode = mode
        self.custom_Kp = Kp
        self.custom_Ki = Ki
        self.custom_Kd = Kd
        self.update_interval = update_interval
        self.smoothing_factor = smoothing_factor
        self.ellipsoid_buffer = ellipsoid_buffer
        self.use_pareto = use_pareto
        self.verbose = verbose
        
        # 新增：详细指标跟踪
        self.track_detailed_metrics = True
        
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
            print(f"\n========== SC-RRT优化算法开始 ({self.dim}D) ==========")
            print(f"算法模式: {self.mode.upper()}")
            print(f"PID参数: Kp={self.custom_Kp}, Ki={self.custom_Ki}, Kd={self.custom_Kd}")
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
        
        # ★★★ 新增：有效采样比例(ESR)统计 ★★★
        samples_in_ellipsoid_A = 0  # 树A的椭球内采样数
        samples_in_ellipsoid_B = 0  # 树B的椭球内采样数
        total_samples_A = 0  # 树A总采样尝试数
        total_samples_B = 0  # 树B总采样尝试数
        
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
            
            # 交汇点更新
            adaptive_interval = max(20, min(100, int(50 * (1 - iterations / self.max_iterations))))
            if iterations % adaptive_interval == 0 and sizeA > 1 and sizeB > 1:
                meet_point_new = self._calculate_meet_point(
                    treeA[:sizeA], treeB[:sizeB], meet_point
                )
                meet_point = self.smoothing_factor * meet_point_old + \
                            (1 - self.smoothing_factor) * meet_point_new
                meet_point_old = meet_point.copy()
                
                # 更新双椭球体参数（使用PID状态）
                (c_best_A, c_best_B, c_min_A, c_min_B, 
                 prev_error_A, integral_error_A, prev_error_B, integral_error_B) = \
                    self._calculate_ellipsoid_params(
                        treeA[:sizeA], treeB[:sizeB], meet_point,
                        prev_error_A, integral_error_A, prev_error_B, integral_error_B
                    )
            
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
            
            # ★★★ 新ESR统计：记录椭球引导采样数 ★★★
            if in_ellipsoid_A:
                samples_in_ellipsoid_A += 1  # 椭球引导采样计数
            
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
            
            # ★★★ 新ESR统计：记录椭球引导采样数 ★★★
            if in_ellipsoid_B:
                samples_in_ellipsoid_B += 1  # 椭球引导采样计数
            
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
            'custom_Kp': self.custom_Kp,
            'custom_Ki': self.custom_Ki,
            'custom_Kd': self.custom_Kd,
            'convergence_history': convergence_history,
            # 新增详细指标
            'avg_tracking_error': np.mean(tracking_errors) if len(tracking_errors) > 0 else np.inf,
            'oscillation_count': oscillation_count,
            'oscillation_rate': oscillation_count / max(1, iterations),
            'failure_mode': failure_mode,
            'ellipsoid_volumes': ellipsoid_volumes,
            'final_ellipsoid_volume': ellipsoid_volumes[-1] if len(ellipsoid_volumes) > 0 else np.inf,
            # ★★★ 新增：有效采样比例(ESR)核心指标 ★★★
            'effective_sampling_ratio': (samples_in_ellipsoid_A + samples_in_ellipsoid_B) / max(1, total_samples_A + total_samples_B),
            'samples_in_ellipsoid': samples_in_ellipsoid_A + samples_in_ellipsoid_B,
            'total_samples_attempted': total_samples_A + total_samples_B,
            'esr_tree_A': samples_in_ellipsoid_A / max(1, total_samples_A),
            'esr_tree_B': samples_in_ellipsoid_B / max(1, total_samples_B)
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
        使用PID控制采样策略（修复版：确保No_PID与PID模式可公平对比）
        
        Returns:
            sample: 采样点
            current_error: 当前误差
            integral_error: 积分误差
            in_ellipsoid: 采样点是否在真实约束椭球内（用于ESR统计）
        
        核心修复：
        1. No_PID模式使用固定基线策略（固定goal_bias_prob=0.10），而非退化为纯随机
        2. 使用 _is_in_ellipsoid 真实校验样本是否落入椭球，确保ESR统计准确
        3. PID模式通过Kp/Ki/Kd自适应调整goal_bias_prob，展现控制效果
        """
        # 计算焦点（用于椭球采样和ESR校验）
        if tree_name == 'A':
            focus1, focus2 = self.start, target
            c_estimated = np.linalg.norm(target - self.start)
        else:
            focus1, focus2 = target, self.goal
            c_estimated = np.linalg.norm(self.goal - target)
        
        # 计算当前误差
        if len(tree) > 1:
            distances = np.linalg.norm(tree[:, :self.dim] - target, axis=1)
            min_dist = np.min(distances)
            env_diagonal = np.linalg.norm(self.goal - self.start)
            current_error = min_dist / (env_diagonal + 1e-10)
        else:
            current_error = 1.0
        
        # ★★★ 核心修复：No_PID使用固定基线，PID使用自适应控制 ★★★
        if self.mode == 'no_pid':
            # No_PID基线：固定低goal_bias（0.10），不进行PID计算
            pid_control = 0.0
            goal_bias_prob = 0.10  # 固定10%椭球采样概率作为公平基线
        else:
            # PID控制计算
            P_term = self.custom_Kp * current_error
            
            integral_error += current_error * 0.05
            integral_error = np.clip(integral_error, -2.0, 2.0)
            I_term = self.custom_Ki * integral_error
            
            error_derivative = current_error - prev_error
            D_term = self.custom_Kd * error_derivative
            
            pid_control = P_term + I_term + D_term
            
            # 映射PID输出到goal_bias_prob [0.15, 0.40]
            goal_bias_prob = 0.15 + 0.25 * pid_control
            goal_bias_prob = np.clip(goal_bias_prob, 0.05, 0.40)
        
        # 计算椭球采样半径
        c_for_sampling = c_best if c_best < np.inf else c_estimated * 4.0
        
        rand_val = np.random.rand()
        
        # ==================== 策略1：椭球内采样 ====================
        if rand_val < goal_bias_prob:
            ellipsoid_scale = 1.5 + (1.0 * (1.0 - goal_bias_prob))
            adjusted_c = c_for_sampling * ellipsoid_scale
            
            sample = sample_in_ellipsoid(focus1, focus2, adjusted_c, self.dim)
            
            if sample is not None:
                # ★ 关键修复：使用_is_in_ellipsoid真实校验ESR
                in_ellipsoid = self._is_in_ellipsoid(sample, focus1, focus2, adjusted_c)
                return sample, current_error, integral_error, in_ellipsoid
            # 如果椭球采样失败，继续到下一策略
        
        # ==================== 策略2：定向偏置采样 ====================
        if rand_val < min(1.0, goal_bias_prob + 0.30):
            base_point = self.start if tree_name == 'A' else self.goal
            direction = target - base_point
            direction_norm = np.linalg.norm(direction)
            
            if direction_norm > 1e-6:
                direction = direction / direction_norm
                
                # 调整随机噪声幅度（更保守）
                randomness = (1.0 - goal_bias_prob) * 150.0
                random_offset = np.random.randn(self.dim) * randomness
                
                distance_factor = np.random.uniform(0.2, 0.8) * goal_bias_prob * direction_norm
                biased_sample = base_point + direction * distance_factor + random_offset
                
                # 边界裁剪
                if self.dim == 2:
                    biased_sample = np.clip(biased_sample,
                                          [self.bounds[0], self.bounds[2]],
                                          [self.bounds[1], self.bounds[3]])
                else:
                    biased_sample = np.clip(biased_sample,
                                          [self.bounds[0], self.bounds[2], self.bounds[4]],
                                          [self.bounds[1], self.bounds[3], self.bounds[5]])
                
                # 校验是否碰巧落入椭球
                in_ellipsoid = self._is_in_ellipsoid(biased_sample, focus1, focus2, c_for_sampling * 2.0)
                return biased_sample, current_error, integral_error, in_ellipsoid
        
        # ==================== 策略3：完全随机采样 ====================
        sample = sample_point(self.bounds, self.dim)
        # 校验随机样本是否碰巧落入椭球（概率极低但需统计）
        in_ellipsoid = self._is_in_ellipsoid(sample, focus1, focus2, c_for_sampling * 2.0)
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
