function path = extractBidirectionalPath(treeA, treeB, nodeA_idx, nodeB_idx, dim)
% extractBidirectionalPath - 从双向树中提取完整路径（增强版：去除冗余）
%
% 输入:
%   treeA      - 树A（从起点生长） [N×(dim+4)]
%   treeB      - 树B（从终点生长） [M×(dim+4)]
%   nodeA_idx  - 树A中的连接节点索引
%   nodeB_idx  - 树B中的连接节点索引
%   dim        - 空间维度
%
% 输出:
%   path - 完整路径 [K×dim]，从起点到终点

% 参数验证
if nodeA_idx < 1 || nodeA_idx > size(treeA, 1)
    error('nodeA_idx索引越界: %d (树A大小: %d)', nodeA_idx, size(treeA, 1));
end
if nodeB_idx < 1 || nodeB_idx > size(treeB, 1)
    error('nodeB_idx索引越界: %d (树B大小: %d)', nodeB_idx, size(treeB, 1));
end

% 从树A回溯路径（起点 -> 连接点A）
pathA = [];
current_idx = nodeA_idx;
visitedA = false(size(treeA, 1), 1);  % 防止循环引用

while current_idx > 0
    % 检查循环
    if visitedA(current_idx)
        warning('检测到树A中的循环引用，节点%d', current_idx);
        break;
    end
    visitedA(current_idx) = true;
    
    pathA = [treeA(current_idx, 1:dim); pathA]; %#ok<AGROW>
    parent_idx = treeA(current_idx, dim+1);
    
    if parent_idx == 0
        break;
    end
    
    % 验证父节点索引
    if parent_idx < 1 || parent_idx > size(treeA, 1)
        warning('树A父节点索引异常: %d (当前节点: %d)', parent_idx, current_idx);
        break;
    end
    
    current_idx = parent_idx;
end

% 从树B回溯路径（连接点B -> 终点）
pathB = [];
current_idx = nodeB_idx;
visitedB = false(size(treeB, 1), 1);  % 防止循环引用

while current_idx > 0
    % 检查循环
    if visitedB(current_idx)
        warning('检测到树B中的循环引用，节点%d', current_idx);
        break;
    end
    visitedB(current_idx) = true;
    
    pathB = [pathB; treeB(current_idx, 1:dim)]; %#ok<AGROW>
    parent_idx = treeB(current_idx, dim+1);
    
    if parent_idx == 0
        break;
    end
    
    % 验证父节点索引
    if parent_idx < 1 || parent_idx > size(treeB, 1)
        warning('树B父节点索引异常: %d (当前节点: %d)', parent_idx, current_idx);
        break;
    end
    
    current_idx = parent_idx;
end

% 注意：pathB已经是正确顺序（连接点B -> 终点），不需要翻转
% 因为树B的parent关系是：终点(root) <- ... <- 连接点B
% 回溯得到：连接点B -> ... -> 终点

% 智能合并：处理两个连接点之间的间隙
if ~isempty(pathA) && ~isempty(pathB)
    connect_point_A = pathA(end, :);      % 树A的连接点
    connect_point_B = pathB(1, :);         % 树B的连接点
    gap_dist = norm(connect_point_A - connect_point_B);
    
    if gap_dist < 1e-6
        % 情况1：两个连接点重合，去除重复
        path = [pathA; pathB(2:end, :)];
    elseif gap_dist < 1.0
        % 情况2：连接点非常接近（<1米），直接连接
        path = [pathA; pathB];
    else
        % 情况3：连接点有明显间隙，插入中间点使路径平滑
        % 在两个连接点之间线性插值，每隔1米一个点
        num_interp = ceil(gap_dist / 1.0);
        interp_points = zeros(num_interp - 1, dim);
        for k = 1:(num_interp - 1)
            t = k / num_interp;
            interp_points(k, :) = (1-t) * connect_point_A + t * connect_point_B;
        end
        path = [pathA; interp_points; pathB];
    end
else
    path = [pathA; pathB];
end

% 最终优化：去除路径中所有距离过近的点（冗余点）
if size(path, 1) > 2
    dists = vecnorm(diff(path), 2, 2);
    valid_rows = [true; dists > 1e-6];  % 保留第一个点和距离大于容差的点
    path = path(valid_rows, :);
end

% 最终验证
if isempty(path)
    error('构建的路径为空！检查树结构是否正确');
end

end
