function sample = samplePoint(bounds, goalBias)
% samplePoint - 基础采样函数
%
% 输入:
%   bounds   - 边界 [xmin xmax ymin ymax ...]
%   goalBias - 目标偏置点（可选）
%
% 输出:
%   sample - 采样点 [1×m]

m = length(bounds) / 2;
sample = zeros(1, m);

for i = 1:m
    sample(i) = bounds(2*i-1) + rand * (bounds(2*i) - bounds(2*i-1));
end

end
