function sample = sampleInEllipsoid(focus1, focus2, cBest, bounds, m)
% sampleInEllipsoid - 高性能超椭球体内采样（优化版）

% 确保焦点为行向量
focus1 = focus1(:)';
focus2 = focus2(:)';

% 快速退出：cBest无穷大，使用均匀采样
if isinf(cBest)
    sample = zeros(1, m);
    for i = 1:m
        sample(i) = bounds(2*i-1) + rand * (bounds(2*i) - bounds(2*i-1));
    end
    return;
end

cMin = norm(focus2 - focus1);

if cBest < cMin * 1.001
    sample = zeros(1, m);
    for i = 1:m
        sample(i) = bounds(2*i-1) + rand * (bounds(2*i) - bounds(2*i-1));
    end
    return;
end

% 椭球参数
a = cBest / 2;
c_focal = cMin / 2;
b_squared = a*a - c_focal*c_focal;

if b_squared < 1e-10
    sample = zeros(1, m);
    for i = 1:m
        sample(i) = bounds(2*i-1) + rand * (bounds(2*i) - bounds(2*i-1));
    end
    return;
end

b = sqrt(b_squared);
center = (focus1 + focus2) / 2;

% 旋转矩阵（预计算）
if cMin > 1e-10
    eVec = (focus2 - focus1) / cMin;
else
    eVec = zeros(1, m); eVec(1) = 1;
end

% 确保eVec是行向量
eVec = eVec(:)';

if m == 2
    ct = eVec(1); st = eVec(2);
    % 直接内联旋转，避免矩阵乘法
    % R = [ct -st; st ct], L = diag([a, b])
    
    for attempt = 1:20  % 减少最大尝试次数
        % 单位圆内采样
        x = randn(2, 1);
        x = x / norm(x);
        r = sqrt(rand);  % 2D情况下的均匀分布
        x = r * x;
        
        % 直接计算变换后的坐标（内联R*L*x + center）
        lx = a * x(1);
        ly = b * x(2);
        sx = ct * lx - st * ly + center(1);
        sy = st * lx + ct * ly + center(2);
        
        % 边界检查
        if sx >= bounds(1) && sx <= bounds(2) && sy >= bounds(3) && sy <= bounds(4)
            sample = [sx, sy];
            return;
        end
    end
else
    % 3D旋转矩阵（修复向量维度问题）
    eVec_col = eVec(:);  % 转换为列向量用于计算
    x_axis = [1; 0; 0];
    rotation_axis = cross(x_axis, eVec_col);
    rotation_axis_norm = norm(rotation_axis);
    
    if rotation_axis_norm < 1e-10
        if dot(x_axis, eVec_col) > 0
            R = eye(3);
        else
            R = [-1 0 0; 0 1 0; 0 0 -1];
        end
    else
        k = rotation_axis / rotation_axis_norm;
        cos_angle = max(-1, min(1, dot(x_axis, eVec_col)));
        angle_val = acos(cos_angle);
        K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
        R = eye(3) + sin(angle_val)*K + (1-cos(angle_val))*(K*K);
    end
    
    L = diag([a, b, b]);
    RL = R * L;  % 预乘
    
    for attempt = 1:20
        x = randn(3, 1);
        x = x / norm(x);
        r = rand^(1/3);
        x = r * x;
        
        sample_col = RL * x + center(:);
        sample = sample_col';
        
        if all(sample >= bounds(1:2:end)) && all(sample <= bounds(2:2:end))
            return;
        end
    end
end

% 采样失败，返回均匀采样
sample = zeros(1, m);
for i = 1:m
    sample(i) = bounds(2*i-1) + rand * (bounds(2*i) - bounds(2*i-1));
end

end
