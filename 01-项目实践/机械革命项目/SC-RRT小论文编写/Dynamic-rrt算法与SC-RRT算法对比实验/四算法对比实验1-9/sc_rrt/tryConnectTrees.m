function [success, connect_idx] = tryConnectTrees(point, tree, obstacles, dim, threshold)
% tryConnectTrees - 高性能树连接尝试（向量化优化版）

success = false;
connect_idx = 0;
n = size(tree, 1);

if n == 0
    return;
end

% 向量化距离计算
diffs = bsxfun(@minus, tree(1:n, 1:dim), point);
dists = sqrt(sum(diffs .* diffs, 2));
[min_dist, nearest_idx] = min(dists);

% 阈值检查后再做碰撞检测
if min_dist < threshold
    if isCollisionFree(tree(nearest_idx, 1:dim), point, obstacles, dim)
        success = true;
        connect_idx = nearest_idx;
    end
end

end
