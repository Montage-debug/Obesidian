function path_opt = shortcutPath(path, obstacles, dim, max_iter)
% shortcutPath - 路径捷径优化
%
% 输入:
%   path       - 原始路径 [N×dim]
%   obstacles  - 障碍物列表
%   dim        - 空间维度
%   max_iter   - 最大迭代次数 (默认: 5)
%
% 输出:
%   path_opt   - 优化后的路径 [M×dim], M <= N
%
% 功能:
%   尝试跳过中间节点直接连接远端节点，减少路径长度
%
% 算法:
%   1. 从路径的第i个点开始
%   2. 尝试直接连接到第j个点 (j > i+1)
%   3. 如果连接无碰撞，删除中间节点
%   4. 重复直到无法优化
%
% 作者: SC-RRT优化团队
% 日期: 2025-12-13

    if nargin < 4
        max_iter = 5;
    end
    
    path_opt = path;
    
    for iter = 1:max_iter
        improved = false;
        i = 1;
        
        while i < size(path_opt, 1) - 1
            % 从最远端开始尝试
            for j = size(path_opt, 1):-1:i+2
                % 尝试直接连接 i 和 j
                if isCollisionFree(path_opt(i,:), path_opt(j,:), obstacles, dim)
                    % 可以跳过中间节点
                    path_opt = [path_opt(1:i,:); path_opt(j:end,:)];
                    improved = true;
                    break;
                end
            end
            i = i + 1;
        end
        
        % 如果本轮没有改进，退出
        if ~improved
            break;
        end
    end
end
