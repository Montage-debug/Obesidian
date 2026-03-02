function [best_idx_A, best_idx_B, best_cost] = findBestConnection(treeA, treeB, obstacles, dim, max_check)
% findBestConnection - 高性能最优连接搜索（向量化优化版）

    if nargin < 5, max_check = 50; end
    
    nA = size(treeA, 1);
    nB = size(treeB, 1);
    checkA = min(nA, max_check);
    checkB = min(nB, max_check);
    
    best_cost = inf;
    best_idx_A = [];
    best_idx_B = [];
    
    % 按代价排序，取前max_check个
    [~, sorted_A] = sort(treeA(1:nA, dim+2));
    [~, sorted_B] = sort(treeB(1:nB, dim+2));
    
    sel_A = sorted_A(1:checkA);
    sel_B = sorted_B(1:checkB);
    
    % 提取坐标和代价
    posA = treeA(sel_A, 1:dim);
    costA = treeA(sel_A, dim+2);
    posB = treeB(sel_B, 1:dim);
    costB = treeB(sel_B, dim+2);
    
    % 向量化计算所有距离矩阵 [checkA × checkB]
    % 使用展开的平方距离
    distMatrix = zeros(checkA, checkB);
    for d = 1:dim
        diff_d = bsxfun(@minus, posA(:, d), posB(:, d)');
        distMatrix = distMatrix + diff_d .* diff_d;
    end
    distMatrix = sqrt(distMatrix);
    
    % 计算总代价矩阵
    totalCost = bsxfun(@plus, costA, costB') + distMatrix;
    
    % 找最小代价的候选对
    [sorted_costs, sorted_linear] = sort(totalCost(:));
    
    for k = 1:length(sorted_linear)
        if sorted_costs(k) >= best_cost
            break;  % 后面的都更大
        end
        
        [i, j] = ind2sub([checkA, checkB], sorted_linear(k));
        
        if isCollisionFree(posA(i, :), posB(j, :), obstacles, dim)
            best_cost = sorted_costs(k);
            best_idx_A = sel_A(i);
            best_idx_B = sel_B(j);
            return;  % 找到最优无碰撞连接
        end
    end
end
