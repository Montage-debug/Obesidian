function x_rand = SampleEllipsoid(xa, xb, bounds, c_hat, m)
% SampleEllipsoid - 在以xa和xb为焦点的超椭球内均匀采样
%
% 论文依据: Section 3.1 - Informed subset定义为超椭球
%   ζ(x) = {x ∈ X_free : ||x-xa|| + ||x-xb|| ≤ c_hat}
%
% 超椭球参数:
%   - 焦点: xa (start), xb (goal)
%   - 主轴直径: c_hat (F̂(c)的估计值)
%   - 焦距: f = ||xb - xa||
%   - 半主轴: a = c_hat / 2
%   - 半短轴: b = sqrt(a^2 - (f/2)^2)
%
% 输入:
%   xa     - 焦点1 (当前start) [1×m]
%   xb     - 焦点2 (goal) [1×m]
%   bounds - 空间边界 [1×2m]
%   c_hat  - 超椭球主轴直径 (F̂(c))
%   m      - 空间维度
%
% 输出:
%   x_rand - 采样点 [1×m]

% 1. 计算超椭球参数
center = (xa + xb) / 2;  % 椭球中心
f = norm(xb - xa);       % 焦距

% 半主轴长度
a = c_hat / 2;

% 检查c_hat是否合理（必须大于焦距）
if c_hat < f || c_hat == inf
    % 如果c_hat不合理，退化为全空间采样
    x_rand = SampleFreeSpace(bounds, m);
    return;
end

% 半短轴长度
b = sqrt(a^2 - (f/2)^2);

% 如果b太小，可能导致数值问题
if b < 1e-6
    x_rand = SampleFreeSpace(bounds, m);
    return;
end

% 2. 构造旋转矩阵（将主轴对齐到xa-xb方向）
if f < 1e-6
    % xa和xb几乎重合，使用单位矩阵
    C = eye(m);
else
    % 主轴方向（单位向量）
    v1 = (xb - xa) / f;
    
    % 构造正交矩阵
    if m == 2
        % 2D情况
        C = [v1(1), -v1(2);
             v1(2),  v1(1)];
    elseif m == 3
        % 3D情况：使用Gram-Schmidt正交化
        % 选择一个不平行于v1的向量
        if abs(v1(1)) < 0.9
            v_temp = [1; 0; 0];
        else
            v_temp = [0; 1; 0];
        end
        
        % Gram-Schmidt
        v2 = v_temp - dot(v_temp, v1') * v1';
        v2 = v2 / norm(v2);
        
        v3 = cross(v1, v2);
        v3 = v3 / norm(v3);
        
        C = [v1; v2; v3]';
    else
        % 高维情况：使用QR分解构造正交基
        A = randn(m, m);
        A(:, 1) = v1';
        [C, ~] = qr(A);
    end
end

% 3. 在单位球内采样并变换到椭球
max_attempts = 100;
for attempt = 1:max_attempts
    % 在单位球内均匀采样
    u = randn(m, 1);
    u = u / norm(u);  % 单位方向
    r = rand^(1/m);   % 半径（保证体积均匀）
    u = u * r;
    
    % 变换到椭球坐标系
    % 对角缩放矩阵：第一维是a，其余维是b
    L = diag([a, repmat(b, 1, m-1)]);
    
    % 变换：旋转 + 缩放 + 平移
    x_rand = (C * L * u)' + center;
    
    % 检查是否在bounds内
    in_bounds = true;
    for i = 1:m
        if x_rand(i) < bounds(2*i-1) || x_rand(i) > bounds(2*i)
            in_bounds = false;
            break;
        end
    end
    
    if in_bounds
        return;
    end
end

% 如果多次尝试都在bounds外，使用全空间采样
x_rand = SampleFreeSpace(bounds, m);

end

function x_rand = SampleFreeSpace(bounds, m)
% 在自由空间内均匀采样（退化策略）
x_rand = zeros(1, m);
for i = 1:m
    x_rand(i) = bounds(2*i-1) + rand * (bounds(2*i) - bounds(2*i-1));
end
end
