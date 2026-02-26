function path = extractBidirectionalPath(treeA, treeB, nodeA_idx, nodeB_idx, dim)
% extractBidirectionalPath - 高性能双向路径提取（优化版）

% 参数验证
if nodeA_idx < 1 || nodeA_idx > size(treeA, 1)
    error('nodeA_idx索引越界: %d (树A大小: %d)', nodeA_idx, size(treeA, 1));
end
if nodeB_idx < 1 || nodeB_idx > size(treeB, 1)
    error('nodeB_idx索引越界: %d (树B大小: %d)', nodeB_idx, size(treeB, 1));
end

% 从树A回溯路径（先收集索引，再一次性提取坐标）
idxListA = zeros(size(treeA, 1), 1);
countA = 0;
current_idx = nodeA_idx;
maxSteps = size(treeA, 1);

while current_idx > 0 && countA < maxSteps
    countA = countA + 1;
    idxListA(countA) = current_idx;
    parent_idx = treeA(current_idx, dim+1);
    if parent_idx == 0 || parent_idx < 1 || parent_idx > size(treeA, 1)
        break;
    end
    current_idx = parent_idx;
end

% 反序得到从起点到连接点的路径
pathA = treeA(idxListA(countA:-1:1), 1:dim);

% 从树B回溯路径
idxListB = zeros(size(treeB, 1), 1);
countB = 0;
current_idx = nodeB_idx;
maxSteps = size(treeB, 1);

while current_idx > 0 && countB < maxSteps
    countB = countB + 1;
    idxListB(countB) = current_idx;
    parent_idx = treeB(current_idx, dim+1);
    if parent_idx == 0 || parent_idx < 1 || parent_idx > size(treeB, 1)
        break;
    end
    current_idx = parent_idx;
end

pathB = treeB(idxListB(1:countB), 1:dim);

% 合并路径
if ~isempty(pathA) && ~isempty(pathB)
    gap_dist = norm(pathA(end, :) - pathB(1, :));
    
    if gap_dist < 1e-6
        path = [pathA; pathB(2:end, :)];
    else
        path = [pathA; pathB];
    end
else
    path = [pathA; pathB];
end

end
