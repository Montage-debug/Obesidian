function [nearestIndex, nearestPoint] = findNearPoint(tree, targetPoint)
    % 功能：在树中寻找距离目标最近的节点（支持任意维度）
    % 输入参数:
    %   tree        - 搜索树 [x1, x2, ..., xm, parent_index, actual_cost, F_hat, out_degree]
    %   targetPoint - 目标点 [x1, x2, ..., xm]
    % 输出参数:
    %   nearestIndex - 最近节点索引
    %   nearestPoint - 最近节点坐标 [x1, x2, ..., xm]

    m = length(targetPoint);
    if size(tree, 2) < m + 4
        error('树维度与目标点不匹配');
    end

    % 确保 targetPoint 为行向量
    targetPoint = reshape(targetPoint, 1, m);
    distances = sqrt(sum((tree(:, 1:m) - repmat(targetPoint, size(tree, 1), 1)).^2, 2));
    [~, nearestIndex] = min(distances);
    nearestPoint = tree(nearestIndex, 1:m);
end