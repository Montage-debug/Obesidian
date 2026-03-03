function xBall = RandomInUnitBall(m)
% RandomInUnitBall - 在单位球内均匀采样
%
% 输入:
%   m - 空间维度
%
% 输出:
%   xBall - 单位球内随机点 [m×1]

% 生成单位球面上的随机点
x = randn(m, 1);
x = x / norm(x);

% 生成半径（均匀分布在球内）
r = rand^(1/m);

xBall = r * x;

end
