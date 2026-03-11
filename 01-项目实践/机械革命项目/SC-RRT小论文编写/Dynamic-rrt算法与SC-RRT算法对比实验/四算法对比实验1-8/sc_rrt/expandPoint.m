function newPoint = expandPoint(nearPoint, randPoint, stepSize)
% expandPoint - 从最近点向随机点扩展
%
% 输入:
%   nearPoint - 最近节点坐标 [1×m]
%   randPoint - 随机采样点 [1×m]
%   stepSize  - 扩展步长
%
% 输出:
%   newPoint - 扩展后的新点 [1×m]

% 确保输入是行向量
if size(nearPoint, 1) > size(nearPoint, 2)
    nearPoint = nearPoint';
end
if size(randPoint, 1) > size(randPoint, 2)
    randPoint = randPoint';
end

% 计算方向向量
direction = randPoint - nearPoint;
distance = norm(direction);

% 如果距离为0，直接返回nearPoint
if distance < 1e-10
    newPoint = nearPoint;
    return;
end

% 归一化方向向量
direction = direction / distance;

% 按步长扩展
if distance <= stepSize
    newPoint = randPoint;
else
    newPoint = nearPoint + stepSize * direction;
end

end
