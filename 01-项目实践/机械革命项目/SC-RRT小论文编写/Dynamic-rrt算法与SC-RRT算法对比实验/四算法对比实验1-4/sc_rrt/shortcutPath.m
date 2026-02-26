function path_opt = shortcutPath(path, obstacles, dim, max_iter)
% shortcutPath - 路径捷径优化（增强版）
%
% 输入:
%   path       - 原始路径 [N×dim]
%   obstacles  - 障碍物列表
%   dim        - 空间维度
%   max_iter   - 贪心迭代次数 (默认: 8)
%
% 输出:
%   path_opt   - 优化后的路径 [M×dim], M <= N
%
% 功能:
%   阶段1: 贪心正向shortcut - 从每个点尝试跳到最远可达点
%   阶段2: 随机对shortcut - 随机选取两个不相邻节点尝试直连
%
% 作者: SC-RRT优化团队
% 日期: 2025-12-14

    if nargin < 4
        max_iter = 8;
    end
    
    path_opt = path;
    
    %% 阶段1: 贪心正向shortcut
    for iter = 1:max_iter
        improved = false;
        i = 1;
        
        while i < size(path_opt, 1) - 1
            % 从最远端开始尝试
            for j = size(path_opt, 1):-1:i+2
                if isCollisionFree(path_opt(i,:), path_opt(j,:), obstacles, dim)
                    path_opt = [path_opt(1:i,:); path_opt(j:end,:)];
                    improved = true;
                    break;
                end
            end
            i = i + 1;
        end
        
        if ~improved
            break;
        end
    end
    
    %% 阶段2: 随机对shortcut（全局优化）
    n = size(path_opt, 1);
    if n < 4
        return;
    end
    
    num_random_trials = min(n * 5, 200);  % 尝试次数与路径节点数成正比
    
    for trial = 1:num_random_trials
        n = size(path_opt, 1);
        if n < 4, break; end
        
        % 随机选两个不相邻索引
        i = randi(n - 2);
        j = i + randi(n - i - 1) + 1;  % j > i+1
        if j > n, continue; end
        
        % 检查直连是否更短且无碰撞
        direct_dist = norm(path_opt(j,:) - path_opt(i,:));
        seg_dist = sum(vecnorm(diff(path_opt(i:j,:)), 2, 2));
        
        if direct_dist < seg_dist * 0.99  % 至少缩短1%
            if isCollisionFree(path_opt(i,:), path_opt(j,:), obstacles, dim)
                path_opt = [path_opt(1:i,:); path_opt(j:end,:)];
            end
        end
    end
end
