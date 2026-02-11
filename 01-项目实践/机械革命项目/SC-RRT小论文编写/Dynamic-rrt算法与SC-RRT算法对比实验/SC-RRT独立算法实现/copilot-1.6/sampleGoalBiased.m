function randomPoint = sampleGoalBiased(goalPoint, goalBiasRadius, bounds)
    % 功能：在目标点附近使用高斯分布采样随机点（支持任意维度）
    % 输入参数:
    %   goalPoint      - 目标点坐标 [x1, x2, ...]
    %   goalBiasRadius - 目标偏置半径（高斯分布标准差）
    %   bounds         - 工作空间边界 [x_min, x_max, y_min, y_max, ...]
    % 输出参数:
    %   randomPoint    - 采样点 [x1, x2, ...]

    m = length(goalPoint);
    if length(bounds) ~= 2*m
        error('bounds 必须包含 %d 个元素', 2*m);
    end

    % 确保 goalPoint 为行向量
    goalPoint = reshape(goalPoint, 1, m);

    % 使用高斯分布采样
    randomPoint = goalPoint + goalBiasRadius * randn(1, m);

    % 边界裁剪
    for i = 1:m
        minBound = bounds(2*i-1);
        maxBound = bounds(2*i);
        randomPoint(i) = max(minBound, min(maxBound, randomPoint(i)));
    end
end