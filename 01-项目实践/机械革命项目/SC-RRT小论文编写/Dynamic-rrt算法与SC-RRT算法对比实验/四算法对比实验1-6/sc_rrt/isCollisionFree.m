function collision = isCollisionFree(point1, point2, obstacles, m)
% isCollisionFree - 高性能碰撞检测（解析线段-球体检测）
%
% 输入:
%   point1    - 起点 [1×m]
%   point2    - 终点 [1×m]
%   obstacles - 障碍物矩阵 [N×(m+1)]
%   m         - 空间维度
%
% 输出:
%   collision - true表示无碰撞，false表示有碰撞

% 如果没有障碍物，直接返回
if isempty(obstacles)
    collision = true;
    return;
end

% 使用解析线段-球体碰撞检测（无需逐点采样）
% 线段参数化: P(t) = point1 + t * d, t ∈ [0, 1]
d = point2 - point1;
d_sq = d * d';  % dot(d,d) 标量

if m == 2
    centers = obstacles(:, 1:2);
    radii = obstacles(:, 3);
else
    centers = obstacles(:, 1:3);
    radii = obstacles(:, 4);
end

% f = point1 - center (向量化)
f = bsxfun(@minus, point1(1:m), centers);  % [N×m]

if d_sq < 1e-12
    % 点退化情况
    dist_sq = sum(f .* f, 2);
    if any(dist_sq < radii .* radii)
        collision = false;
    else
        collision = true;
    end
    return;
end

% 二次方程 a*t^2 + b*t + c = 0 求解
% a = dot(d,d) 对所有障碍物相同
b = 2 * (f * d');      % [N×1]
c = sum(f .* f, 2) - radii .* radii;  % [N×1]

discriminant = b .* b - 4 * d_sq * c;

% 只看判别式>=0的（有实数交点）
idx = discriminant >= 0;
if ~any(idx)
    collision = true;
    return;
end

% 计算交点参数t
sqrt_disc = sqrt(discriminant(idx));
b_sel = b(idx);
inv_2a = 1 / (2 * d_sq);
t1 = (-b_sel - sqrt_disc) * inv_2a;
t2 = (-b_sel + sqrt_disc) * inv_2a;

% 线段在 t∈[0,1], 球体交集在 [t1,t2]
% 有碰撞条件: t1 <= 1 且 t2 >= 0
if any(t1 <= 1 & t2 >= 0)
    collision = false;
else
    collision = true;
end

end
