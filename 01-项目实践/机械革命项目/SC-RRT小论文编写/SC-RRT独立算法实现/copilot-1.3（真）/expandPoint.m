% expandPoint.m
function newP = expandPoint(nearest, target, stepSize)
    % 功能：Steer 函数，从 nearest 向 target 按步长扩展
    dir = target - nearest;                             % 方向向量
    dist = norm(dir);                                   % 距离
    if dist > stepSize
        newP = nearest + (dir / dist) * stepSize;       % 按步长移动
    else
        newP = target;                                  % 直接到达采样点
    end
end
