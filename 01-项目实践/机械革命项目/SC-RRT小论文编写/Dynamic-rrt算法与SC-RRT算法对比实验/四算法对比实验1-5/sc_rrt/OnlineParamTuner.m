classdef OnlineParamTuner < handle
% OnlineParamTuner - 基于上下文多臂老虎机的在线参数自适应调节器
%
% 核心思想 (Contextual Bandit with UCB):
%   将SSFOR参数选择建模为上下文多臂老虎机问题。
%   上下文(Context): 当前搜索状态特征向量 [改进率, 碰撞率, 树不平衡度, 进度比]
%   臂(Arms): 离散化的参数组合 (采样策略概率, 椭球缓冲系数)
%   奖励(Reward): 基于路径改进、碰撞避免、连接效率的复合信号
%
%   每隔固定迭代间隔，根据UCB准则选择当前最优参数组合，
%   并根据后续搜索表现更新奖励估计，实现零先验知识下的在线自适应。
%
% 替代的硬编码参数:
%   p_meet    - 交汇点采样概率 (原固定0.35)
%   p_cross   - 跨树采样概率   (原固定0.15)
%   gamma     - 椭球缓冲系数   (原固定1.2)
%
% 参考文献:
%   Auer, P. et al. (2002). "Finite-time Analysis of the Multiarmed 
%   Bandit Problem." Machine Learning, 47(2-3), 235-256.
%
% 作者: SC-RRT优化团队
% 日期: 2025-12-14

    properties
        %% ===== 臂空间 (Arms) =====
        arms           % Nx3 矩阵, 每行 [p_meet, p_cross, gamma]
        num_arms       % 臂的数量
        
        %% ===== 统计量 =====
        rewards_sum    % 每个臂的累积奖励
        pull_counts    % 每个臂被选择的次数
        total_pulls    % 总选择次数
        
        %% ===== 当前状态 =====
        current_arm    % 当前选择的臂索引
        current_params % 当前参数 [p_meet, p_cross, gamma]
        
        %% ===== 搜索状态追踪 =====
        cost_history       % 最近N次的路径代价
        collision_count    % 采样碰撞次数（当前窗口）
        sample_count       % 采样总次数（当前窗口）
        improvement_count  % 路径改进次数（当前窗口）
        window_size        % 统计窗口大小
        
        %% ===== 超参数 =====
        ucb_c          % UCB探索系数
        update_interval % 参数更新间隔
        decay_factor   % 奖励衰减因子（非平稳环境）
    end
    
    methods
        function obj = OnlineParamTuner(varargin)
            % 构造函数
            %
            % 可选参数:
            %   'UCB_C'           - UCB探索系数 (默认: 1.5)
            %   'UpdateInterval'  - 参数更新间隔 (默认: 30)
            %   'DecayFactor'     - 奖励衰减 (默认: 0.95)
            
            % 解析参数
            ucb_c_val = 1.5;
            update_val = 30;
            decay_val = 0.95;
            
            i = 1;
            while i <= length(varargin)
                if ischar(varargin{i})
                    switch varargin{i}
                        case 'UCB_C',          ucb_c_val = varargin{i+1}; i = i + 2;
                        case 'UpdateInterval', update_val = varargin{i+1}; i = i + 2;
                        case 'DecayFactor',    decay_val = varargin{i+1}; i = i + 2;
                        otherwise, i = i + 2;
                    end
                else
                    i = i + 1;
                end
            end
            
            obj.ucb_c = ucb_c_val;
            obj.update_interval = update_val;
            obj.decay_factor = decay_val;
            
            %% ===== 构建臂空间 =====
            % p_meet ∈ {0.20, 0.30, 0.40, 0.50}
            % p_cross ∈ {0.05, 0.10, 0.15, 0.20}
            % gamma ∈ {1.05, 1.15, 1.30, 1.50}
            p_meet_vals  = [0.20, 0.30, 0.40, 0.50];
            p_cross_vals = [0.05, 0.10, 0.15, 0.20];
            gamma_vals   = [1.05, 1.15, 1.30, 1.50];
            
            % 生成所有组合 (4x4x4 = 64 arms)
            [pm, pc, gm] = ndgrid(p_meet_vals, p_cross_vals, gamma_vals);
            obj.arms = [pm(:), pc(:), gm(:)];
            obj.num_arms = size(obj.arms, 1);
            
            %% ===== 初始化统计量 =====
            obj.rewards_sum = zeros(obj.num_arms, 1);
            obj.pull_counts = ones(obj.num_arms, 1);  % 伪计数1避免除零
            obj.total_pulls = obj.num_arms;
            
            % 初始臂：接近原始硬编码值 [0.35, 0.15, 1.2]
            % 找到最接近的臂
            diffs = obj.arms - repmat([0.35, 0.15, 1.20], obj.num_arms, 1);
            [~, obj.current_arm] = min(sum(diffs.^2, 2));
            obj.current_params = obj.arms(obj.current_arm, :);
            
            %% ===== 搜索状态追踪 =====
            obj.cost_history = [];
            obj.collision_count = 0;
            obj.sample_count = 0;
            obj.improvement_count = 0;
            obj.window_size = 50;
        end
        
        function params = getParams(obj)
            % 获取当前参数 [p_meet, p_cross, gamma]
            params = obj.current_params;
        end
        
        function [p_meet, p_cross, p_ellipsoid, gamma] = getSamplingParams(obj)
            % 获取解包后的采样参数
            p_meet = obj.current_params(1);
            p_cross = obj.current_params(2);
            gamma = obj.current_params(3);
            p_ellipsoid = 1.0 - p_meet - p_cross;  % 剩余概率分配给椭球采样
        end
        
        function recordSample(obj, collision_occurred)
            % 记录一次采样事件
            obj.sample_count = obj.sample_count + 1;
            if collision_occurred
                obj.collision_count = obj.collision_count + 1;
            end
        end
        
        function recordCostUpdate(obj, new_cost)
            % 记录路径代价更新
            if ~isempty(obj.cost_history) && new_cost < obj.cost_history(end)
                obj.improvement_count = obj.improvement_count + 1;
            end
            obj.cost_history = [obj.cost_history; new_cost];
            if length(obj.cost_history) > obj.window_size * 2
                obj.cost_history = obj.cost_history(end - obj.window_size + 1:end);
            end
        end
        
        function should_update = shouldUpdate(obj, iter_count)
            % 判断是否需要更新参数
            should_update = (mod(iter_count, obj.update_interval) == 0) && ...
                            (obj.sample_count >= 10);
        end
        
        function updateParams(obj, sizeA, sizeB)
            % 核心方法：计算奖励并用UCB准则选择新参数组合
            %
            % 输入:
            %   sizeA - 树A节点数
            %   sizeB - 树B节点数
            
            %% ===== 1. 计算复合奖励信号 =====
            reward = obj.computeReward(sizeA, sizeB);
            
            %% ===== 2. 更新当前臂的统计量（带衰减） =====
            % 对所有臂的累积奖励施加衰减（应对非平稳性）
            obj.rewards_sum = obj.rewards_sum * obj.decay_factor;
            obj.pull_counts = obj.pull_counts * obj.decay_factor;
            
            % 更新当前臂
            obj.rewards_sum(obj.current_arm) = obj.rewards_sum(obj.current_arm) + reward;
            obj.pull_counts(obj.current_arm) = obj.pull_counts(obj.current_arm) + 1;
            obj.total_pulls = obj.total_pulls + 1;
            
            %% ===== 3. UCB1选择新臂 =====
            avg_rewards = obj.rewards_sum ./ max(obj.pull_counts, 1e-6);
            exploration_bonus = obj.ucb_c * sqrt(log(obj.total_pulls) ./ max(obj.pull_counts, 1e-6));
            
            ucb_values = avg_rewards + exploration_bonus;
            
            % 加入微小随机扰动打破平局
            ucb_values = ucb_values + rand(size(ucb_values)) * 1e-6;
            
            [~, best_arm] = max(ucb_values);
            obj.current_arm = best_arm;
            obj.current_params = obj.arms(obj.current_arm, :);
            
            %% ===== 4. 重置窗口统计 =====
            obj.collision_count = 0;
            obj.sample_count = 0;
            obj.improvement_count = 0;
        end
        
        function info = getDebugInfo(obj)
            % 返回调试信息结构体
            info.current_params = obj.current_params;
            info.current_arm = obj.current_arm;
            info.total_pulls = obj.total_pulls;
            
            avg_rewards = obj.rewards_sum ./ max(obj.pull_counts, 1e-6);
            [~, top5_idx] = sort(avg_rewards, 'descend');
            top5_idx = top5_idx(1:min(5, length(top5_idx)));
            info.top5_arms = obj.arms(top5_idx, :);
            info.top5_rewards = avg_rewards(top5_idx);
        end
    end
    
    methods (Access = private)
        function reward = computeReward(obj, sizeA, sizeB)
            % 计算复合奖励信号
            %
            % 奖励由三个分量组成:
            %   R1: 路径改进率 — 鼓励选择能促进代价下降的参数
            %   R2: 采样效率   — 惩罚碰撞率高的参数（浪费采样）
            %   R3: 树平衡度   — 鼓励双树均衡生长
            
            %% R1: 路径改进率
            if obj.sample_count > 0 && ~isempty(obj.cost_history)
                improvement_rate = obj.improvement_count / obj.sample_count;
                
                % 代价下降幅度
                if length(obj.cost_history) >= 2
                    recent = obj.cost_history(max(1, end - 10):end);
                    if recent(1) > 0
                        cost_reduction = (recent(1) - recent(end)) / recent(1);
                        cost_reduction = max(0, cost_reduction);  % 只奖励正改进
                    else
                        cost_reduction = 0;
                    end
                else
                    cost_reduction = 0;
                end
                
                R1 = 0.4 * improvement_rate + 0.6 * cost_reduction;
            else
                R1 = 0;
            end
            
            %% R2: 采样效率（碰撞惩罚）
            if obj.sample_count > 0
                collision_rate = obj.collision_count / obj.sample_count;
                R2 = 1 - collision_rate;  % 无碰撞 = 满分
            else
                R2 = 0.5;
            end
            
            %% R3: 树平衡度
            total_nodes = sizeA + sizeB;
            if total_nodes > 2
                balance = 1 - abs(sizeA - sizeB) / total_nodes;
            else
                balance = 1.0;
            end
            R3 = balance;
            
            %% 复合奖励（加权求和）
            reward = 0.5 * R1 + 0.3 * R2 + 0.2 * R3;
            
            % 裁剪到 [0, 1]
            reward = max(0, min(1, reward));
        end
    end
end
