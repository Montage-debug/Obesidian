function collision = isCollisionFree(point1, point2, obstacles, m)
% isCollisionFree - 检查路径是否无碰撞
%
% 输入:
%   point1    - 起点 [1×m]
%   point2    - 终点 [1×m]
%   obstacles - 障碍物矩阵 [N×(m+1)]
%   m         - 空间维度
%
% 输出:
%   collision - true表示无碰撞，false表示有碰撞

% 确保输入是行向量
if size(point1, 1) > size(point1, 2)
    point1 = point1';
end
if size(point2, 1) > size(point2, 2)
    point2 = point2';
end

% 如果没有障碍物，直接返回无碰撞
if isempty(obstacles)
    collision = true;
    return;
end

% 计算路径长度和检查点数
pathLength = norm(point2 - point1);
numChecks = ceil(pathLength / 0.5) + 1;  % 每0.5单位检查一次

% 沿路径检查碰撞
for i = 0:numChecks
    t = i / numChecks;
    checkPoint = point1 + t * (point2 - point1);
    
    % 检查与所有障碍物的碰撞
    for j = 1:size(obstacles, 1)
        if m == 2
            center = obstacles(j, 1:2);
            radius = obstacles(j, 3);
        else
            center = obstacles(j, 1:3);
            radius = obstacles(j, 4);
        end
        
        % 如果距离小于半径，有碰撞
        if norm(checkPoint - center) < radius
            collision = false;
            return;
        end
    end
end

% 所有检查点都无碰撞
collision = true;

end
