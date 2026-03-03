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
    n = size(path, 1);
    
    % 最小路径节点检查：至少保留3个节点（起点、中间1个、终点）
    if n <= 3
        return;
    end
    
    %% 阶段1: 贪心正向shortcut（充分优化）
    for iter = 1:max_iter
        improved = false;
        i = 1;
        
        while i < size(path_opt, 1) - 1
            max_j = size(path_opt, 1);
            start_j = min(max_j, i + 2);
            
            found_shortcut = false;
            for j = max_j:-1:start_j
                if isCollisionFree(path_opt(i,:), path_opt(j,:), obstacles, dim)
                    path_opt = [path_opt(1:i,:); path_opt(j:end,:)];
                    improved = true;
                    found_shortcut = true;
                    break;
                end
            end
            
            if ~found_shortcut
                i = i + 1;
            end
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
    
    num_random_trials = min(n * 5, 200);
    
    for trial = 1:num_random_trials
        n = size(path_opt, 1);
        if n < 4, break; end
        
        i = randi(n - 2);
        j = i + randi(n - i - 1) + 1;
        if j > n, continue; end
        
        direct_dist = norm(path_opt(j,:) - path_opt(i,:));
        seg_dist = sum(vecnorm(diff(path_opt(i:j,:)), 2, 2));
        
        % 改进条件：至少缩短2%
        if direct_dist < seg_dist * 0.98
            if isCollisionFree(path_opt(i,:), path_opt(j,:), obstacles, dim)
                path_opt = [path_opt(1:i,:); path_opt(j:end,:)];
            end
        end
    end
    
    %% 阶段3: 最小节点保障 - 确保路径不会过度稀疏
    if size(path_opt, 1) < 4
        path_opt = resamplePathLocal(path_opt, 6);
    end
end

function path_out = resamplePathLocal(path, target_n)
% 等弧长重采样路径到目标节点数
    seg_lens = vecnorm(diff(path), 2, 2);
    cum_lens = [0; cumsum(seg_lens)];
    total_len = cum_lens(end);
    if total_len < 1e-10 || size(path, 1) >= target_n
        path_out = path;
        return;
    end
    t_target = linspace(0, total_len, target_n)';
    d = size(path, 2);
    path_out = zeros(target_n, d);
    path_out(1, :) = path(1, :);
    path_out(end, :) = path(end, :);
    for i = 2:target_n-1
        idx = find(cum_lens <= t_target(i), 1, 'last');
        idx = min(idx, size(path, 1) - 1);
        seg_len = seg_lens(idx);
        if seg_len < 1e-10
            path_out(i, :) = path(idx, :);
        else
            alpha = (t_target(i) - cum_lens(idx)) / seg_len;
            alpha = max(0, min(1, alpha));
            path_out(i, :) = path(idx, :) * (1 - alpha) + path(idx + 1, :) * alpha;
        end
    end
end
