function path_opt = pullPathToOptimal(path, obstacles, dim, max_iters)
% pullPathToOptimal - 弹性带路径拉直优化（Elastic Band思想）
%
% 核心思想: 将路径上的每个中间节点向其相邻节点连线的投影点方向拉动，
% 使路径逐步趋向局部最短路径，同时保持碰撞安全。
%
% 输入:
%   path       - 原始路径 [N×dim]
%   obstacles  - 障碍物列表
%   dim        - 空间维度 (2 或 3)
%   max_iters  - 最大迭代次数 (默认: 20)
%
% 输出:
%   path_opt   - 优化后的路径 [N×dim] (节点数不变，位置优化)
%
% 算法流程:
%   对每个中间节点P(i):
%     1. 计算P(i)在线段P(i-1)→P(i+1)上的投影点Q
%     2. 以递减步长尝试将P(i)移动到Q方向
%     3. 验证移动后路径段的碰撞安全性
%     4. 仅当新路径不变长时接受移动
%   重复多轮直到收敛
%
% 作者: SC-RRT优化团队
% 日期: 2025-12-14

    if nargin < 4
        max_iters = 20;
    end
    
    path_opt = path;
    n = size(path_opt, 1);
    
    if n < 3
        return;
    end
    
    % 记录初始路径长度用于安全检查
    initial_length = sum(vecnorm(diff(path_opt), 2, 2));
    
    for iter = 1:max_iters
        improved = false;
        
        for i = 2:size(path_opt, 1) - 1
            p_prev = path_opt(i-1, :);
            p_curr = path_opt(i, :);
            p_next = path_opt(i+1, :);
            
            % 计算p_curr在线段p_prev→p_next上的投影点
            line_dir = p_next - p_prev;
            line_len = norm(line_dir);
            
            if line_len < 1e-8
                continue;
            end
            
            line_unit = line_dir / line_len;
            
            % 投影参数t，限制在[0, line_len]内
            t = dot(p_curr - p_prev, line_unit);
            t = max(0, min(line_len, t));
            projection = p_prev + t * line_unit;
            
            % 移动方向：当前点 → 投影点
            move_vec = projection - p_curr;
            move_dist = norm(move_vec);
            
            if move_dist < 1e-6
                continue;  % 已经在直线上，无需移动
            end
            
            % 计算当前两段的总长度
            old_cost = norm(p_curr - p_prev) + norm(p_next - p_curr);
            
            % 以递减步长尝试移动
            for alpha = [0.9, 0.7, 0.5, 0.3, 0.15]
                new_pos = p_curr + alpha * move_vec;
                
                % 检查新位置的两段是否无碰撞
                if isCollisionFree(p_prev, new_pos, obstacles, dim) && ...
                   isCollisionFree(new_pos, p_next, obstacles, dim)
                    
                    % 检查新路径是否不变长（允许微小增长1‰）
                    new_cost = norm(new_pos - p_prev) + norm(p_next - new_pos);
                    
                    if new_cost <= old_cost * 1.001
                        path_opt(i, :) = new_pos;
                        improved = true;
                        break;
                    end
                end
            end
        end
        
        % 也进行反向遍历（消除方向偏差）
        if mod(iter, 2) == 0
            for i = size(path_opt, 1) - 1:-1:2
                p_prev = path_opt(i-1, :);
                p_curr = path_opt(i, :);
                p_next = path_opt(i+1, :);
                
                line_dir = p_next - p_prev;
                line_len = norm(line_dir);
                if line_len < 1e-8, continue; end
                
                line_unit = line_dir / line_len;
                t = dot(p_curr - p_prev, line_unit);
                t = max(0, min(line_len, t));
                projection = p_prev + t * line_unit;
                
                move_vec = projection - p_curr;
                move_dist = norm(move_vec);
                if move_dist < 1e-6, continue; end
                
                old_cost = norm(p_curr - p_prev) + norm(p_next - p_curr);
                
                for alpha = [0.9, 0.7, 0.5, 0.3, 0.15]
                    new_pos = p_curr + alpha * move_vec;
                    if isCollisionFree(p_prev, new_pos, obstacles, dim) && ...
                       isCollisionFree(new_pos, p_next, obstacles, dim)
                        new_cost = norm(new_pos - p_prev) + norm(p_next - new_pos);
                        if new_cost <= old_cost * 1.001
                            path_opt(i, :) = new_pos;
                            improved = true;
                            break;
                        end
                    end
                end
            end
        end
        
        if ~improved
            break;  % 收敛，停止迭代
        end
    end
    
    % 安全检查：确保优化后路径不比原始长太多
    final_length = sum(vecnorm(diff(path_opt), 2, 2));
    if final_length > initial_length * 1.02
        path_opt = path;  % 回退到原始路径
    end
end
