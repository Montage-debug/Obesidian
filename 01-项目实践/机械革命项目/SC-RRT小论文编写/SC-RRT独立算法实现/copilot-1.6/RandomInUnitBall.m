function x = RandomInUnitBall(m)
% RandomInUnitBall 单位球内均匀采样点
%
% 输入:
%   m - 空间维度 (2或3)
%
% 输出:
%   x - m维单位球内的随机点 [m×1]
%
% 方法:
%   1. 生成高斯随机方向，得到均匀球面
%   2. 生成随机半径 r ~ U^(1/m) 保证空间均匀
%   3. 返回 x = r * dir

    % 参数验证
    if m < 1 || m > 3
        error('RandomInUnitBall: 维度m不支持。只支持1-3维');
    end
    
    % 随机方向，高斯分布后归一化到球面
    v = randn(m, 1);
    v = v / norm(v);
    
    % 随机半径，服从球体积分布
    r = rand()^(1/m);
    
    % 组合方向和半径
    x = v * r;
end