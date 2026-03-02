function path = buildPathFromTree(tree, goal_idx, dim)
% buildPathFromTree - 从树中回溯构建路径
%
% 输入:
%   tree     - 树矩阵 [N×(dim+4)]
%   goal_idx - 目标节点索引
%   dim      - 空间维度
%
% 输出:
%   path - 路径矩阵 [M×dim]，从起点到终点

if goal_idx <= 0 || goal_idx > size(tree, 1)
    path = [];
    return;
end

% 回溯路径
path_indices = [];
current_idx = goal_idx;

while current_idx > 0
    path_indices = [current_idx; path_indices]; %#ok<AGROW>
    parent_idx = tree(current_idx, dim+1);
    
    if parent_idx == 0
        break;
    end
    
    current_idx = parent_idx;
    
    % 防止无限循环
    if length(path_indices) > size(tree, 1)
        break;
    end
end

% 提取路径坐标
path = tree(path_indices, 1:dim);

end
