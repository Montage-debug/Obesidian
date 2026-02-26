function [best_parent_idx, best_cost] = findBestParent(tree, new_point, nearest_idx, search_radius, obstacles, dim)
% findBestParent - 为新节点寻找最优父节点（RRT*重连策略）
%
% 功能:
%   在搜索半径内寻找能提供最小代价的父节点
%
% 输入:
%   tree          - 树矩阵 [N×(dim+4)]
%   new_point     - 新节点坐标 [1×dim]
%   nearest_idx   - 最近节点索引（默认父节点）
%   search_radius - 搜索半径
%   obstacles     - 障碍物结构体
%   dim           - 空间维度
%
% 输出:
%   best_parent_idx - 最优父节点索引
%   best_cost       - 到新节点的最优代价

n = size(tree, 1);
best_parent_idx = nearest_idx;
best_cost = tree(nearest_idx, dim+2) + norm(new_point - tree(nearest_idx, 1:dim));

% 在搜索半径内寻找候选父节点
for i = 1:n
    dist = norm(tree(i, 1:dim) - new_point);
    
    if dist < search_radius
        % 计算通过节点i到达new_point的代价
        potential_cost = tree(i, dim+2) + dist;
        
        % 如果代价更小且路径无碰撞
        if potential_cost < best_cost
            if isCollisionFree(tree(i, 1:dim), new_point, obstacles, dim)
                best_cost = potential_cost;
                best_parent_idx = i;
            end
        end
    end
end

end
