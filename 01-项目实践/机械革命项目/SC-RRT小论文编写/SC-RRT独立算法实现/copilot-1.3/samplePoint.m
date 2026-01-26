function randomPoint = samplePoint(bounds, goalPoint)
    % 功能：在工作空间内均匀采样随机点，结合目标导向（支持任意维度）
    % 输入参数:
    %   bounds     - 工作空间边界 [x_min, x_max, y_min, y_max, ...]
    %   goalPoint  - 目标点坐标 [x1, x2, ..., xm]
    % 输出参数:
    %   randomPoint - 采样点 [x1, x2, ..., xm]

    m = length(goalPoint);
    if length(bounds) ~= 2*m
        error('bounds 必须包含 %d 个元素', 2*m);
    end

    goalBias = 0.2; % 目标偏置概率
    if rand < goalBias
        % 目标导向采样：在目标点周围均匀采样
        range = bounds(2:2:end) - bounds(1:2:end); % 1xM vector
        offset = range .* (rand(1, m) - 0.5) * 0.1; % 元素-wise 乘法
        randomPoint = goalPoint + offset;
    else
        % 均匀采样
        randomPoint = zeros(1, m);
        for i = 1:m
            minBound = bounds(2*i-1);
            maxBound = bounds(2*i);
            randomPoint(i) = minBound + rand * (maxBound - minBound);
        end
    end

    % 边界检查（确保采样点在边界内）
    for i = 1:m
        randomPoint(i) = max(bounds(2*i-1), min(bounds(2*i), randomPoint(i)));
    end
end