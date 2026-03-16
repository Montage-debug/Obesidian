function sample = sampleGoalBiased(goalPoint, biasRadius, bounds)
% sampleGoalBiased - 目标偏置采样
%
% 输入:
%   goalPoint  - 目标点 [1×m]
%   biasRadius - 偏置半径
%   bounds     - 边界 [xmin xmax ymin ymax ...]
%
% 输出:
%   sample - 采样点 [1×m]

m = length(goalPoint);

% 在目标点周围的球内采样
if m == 2
    angle = 2 * pi * rand;
    r = biasRadius * sqrt(rand);
    sample = goalPoint + r * [cos(angle), sin(angle)];
else
    % 3D球内采样
    theta = 2 * pi * rand;
    phi = acos(2 * rand - 1);
    r = biasRadius * rand^(1/3);
    sample = goalPoint + r * [sin(phi)*cos(theta), sin(phi)*sin(theta), cos(phi)];
end

% 确保在边界内
for i = 1:m
    sample(i) = max(bounds(2*i-1), min(bounds(2*i), sample(i)));
end

end
