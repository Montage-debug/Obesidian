function x_rand = SampleEllipsoid(xa, xb, bounds, c_hat, m)
% SampleEllipsoid - 在以xa和xb为焦点的超椭球内均匀采样
%
% 超椭球参数:
%   - 焦点: xa (start), xb (goal)
%   - 主轴直径: c_hat (F_hat(c)的估计值)
%   - 焦距: f = ||xb - xa||
%   - 半主轴: a = c_hat / 2
%   - 半短轴: b = sqrt(a^2 - (f/2)^2)

center = (xa + xb) / 2;
f = norm(xb - xa);
a = c_hat / 2;

if c_hat < f || c_hat == inf
    x_rand = SampleFreeSpace(bounds, m);
    return;
end

b = sqrt(a^2 - (f/2)^2);

if b < 1e-6
    x_rand = SampleFreeSpace(bounds, m);
    return;
end

if f < 1e-6
    C = eye(m);
else
    v1 = (xb - xa) / f;
    if m == 2
        C = [v1(1), -v1(2);
             v1(2),  v1(1)];
    elseif m == 3
        if abs(v1(1)) < 0.9
            v_temp = [1; 0; 0];
        else
            v_temp = [0; 1; 0];
        end
        v2 = v_temp - dot(v_temp, v1') * v1';
        v2 = v2 / norm(v2);
        v3 = cross(v1, v2);
        v3 = v3 / norm(v3);
        C = [v1; v2; v3]';
    else
        A = randn(m, m);
        A(:, 1) = v1';
        [C, ~] = qr(A);
    end
end

max_attempts = 100;
for attempt = 1:max_attempts
    u = randn(m, 1);
    u = u / norm(u);
    r = rand^(1/m);
    u = u * r;
    L = diag([a, repmat(b, 1, m-1)]);
    x_rand = (C * L * u)' + center;
    
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

x_rand = SampleFreeSpace(bounds, m);

end

function x_rand = SampleFreeSpace(bounds, m)
x_rand = zeros(1, m);
for i = 1:m
    x_rand(i) = bounds(2*i-1) + rand * (bounds(2*i) - bounds(2*i-1));
end
end
