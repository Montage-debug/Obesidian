function sample = sampleInEllipsoid(focus1, focus2, cBest, bounds, m)
% sampleInEllipsoid - 在超椭球体内采样
%
% 输入:
%   focus1 - 第一个焦点 [1×m]
%   focus2 - 第二个焦点 [1×m]
%   cBest  - 椭球约束参数(焦点距离之和)
%   bounds - 边界 [xmin xmax ymin ymax ...]
%   m      - 空间维度
%
% 输出:
%   sample - 采样点 [1×m] (列向量或行向量取决于输入)
%
% 作者: SC-RRT完全优化版
% 日期: 2025-12-12

% 如果cBest无穷大,使用均匀采样
if isinf(cBest)
    sample = uniformSample(bounds, m);
    return;
end

% 计算焦距
cMin = norm(focus2 - focus1);

% 边界检查
if cBest < cMin * 1.001
    sample = uniformSample(bounds, m);
    return;
end

% 计算椭球参数
a = cBest / 2;                      % 半长轴
c_focal = cMin / 2;                 % 半焦距
b_squared = a^2 - c_focal^2;        % b² = a² - c²

% 数值稳定性检查
if b_squared < 1e-10
    sample = uniformSample(bounds, m);
    return;
end

b = sqrt(b_squared);                % 半短轴

% 椭球中心
center = (focus1 + focus2) / 2;

% 主轴方向向量
eVec = (focus2 - focus1) / cMin;

% 计算旋转矩阵
if m == 2
    theta = atan2(eVec(2), eVec(1));
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
else
    % 3D旋转矩阵 (Rodrigues公式)
    x_axis = [1; 0; 0];
    rotation_axis = cross(x_axis(:), eVec(:));
    rotation_axis_norm = norm(rotation_axis);
    
    if rotation_axis_norm < 1e-10
        if dot(x_axis, eVec(:)) > 0
            R = eye(3);
        else
            R = [-1 0 0; 0 1 0; 0 0 -1];
        end
    else
        k = rotation_axis / rotation_axis_norm;
        cos_angle = dot(x_axis, eVec(:));
        cos_angle = max(-1, min(1, cos_angle));
        angle = acos(cos_angle);
        K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
        R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
    end
end

% 在椭球内采样 (最多尝试100次)
max_attempts = 100;
for attempt = 1:max_attempts
    % 单位球内采样
    xBall = RandomInUnitBall(m);
    
    % 变换到椭球空间
    L = diag([a, repmat(b, 1, m-1)]);
    sample_col = R * (L * xBall(:)) + center(:);
    
    % 转换为行向量
    sample = sample_col';
    
    % 验证边界
    if all(sample >= bounds(1:2:end)) && all(sample <= bounds(2:2:end))
        % 验证椭球约束
        dist_sum = norm(sample - focus1) + norm(sample - focus2);
        if dist_sum <= cBest * 1.001
            return;
        end
    end
end

% 如果采样失败,返回均匀采样
sample = uniformSample(bounds, m);

end

%% ========== 辅助函数 ==========
function sample = uniformSample(bounds, m)
% 均匀采样
sample = zeros(1, m);
for i = 1:m
    sample(i) = bounds(2*i-1) + rand * (bounds(2*i) - bounds(2*i-1));
end
end
