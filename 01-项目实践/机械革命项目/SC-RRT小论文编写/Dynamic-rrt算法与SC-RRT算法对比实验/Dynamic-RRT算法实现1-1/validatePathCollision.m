function [isValid, collisionPoints] = validatePathCollision(path, obstacles, m, checkDensity)
% validatePathCollision - 验证路径是否与障碍物碰撞（增强版）
%
% 输入:
%   path         - 路径点 [N×m]
%   obstacles    - 障碍物结构体 (circles/spheres)
%   m            - 维度 (2或3)
%   checkDensity - 每段路径的检查点数量 (默认20)
%
% 输出:
%   isValid         - 路径是否有效（无碰撞）
%   collisionPoints - 碰撞点位置 [K×m]

if nargin < 4
    checkDensity = 20;
end

collisionPoints = [];
isValid = true;

if size(path, 1) < 2
    return;
end

% 安全裕度（避免路径过于接近障碍物）
safetyMarginRatio = 0.02;  % 2%的安全裕度

% 检查每段路径
for i = 1:size(path, 1)-1
    point1 = path(i, 1:m);
    point2 = path(i+1, 1:m);
    
    % 在两点之间进行密集采样检查
    for k = 0:checkDensity
        t = k / checkDensity;
        samplePoint = point1 * (1-t) + point2 * t;
        
        % 检查该点是否在障碍物内或过于接近
        if m == 2
            % 2D圆形障碍物检查
            if isfield(obstacles, 'circles') && ~isempty(obstacles.circles)
                for j = 1:size(obstacles.circles, 1)
                    center = obstacles.circles(j, 1:2);
                    radius = obstacles.circles(j, 3);
                    safetyMargin = radius * safetyMarginRatio;
                    dist = norm(samplePoint - center);
                    
                    if dist < (radius + safetyMargin)
                        isValid = false;
                        collisionPoints = [collisionPoints; samplePoint];
                        return;  % 发现碰撞立即返回
                    end
                end
            end
        else
            % 3D球形障碍物检查
            if isfield(obstacles, 'spheres') && ~isempty(obstacles.spheres)
                for j = 1:size(obstacles.spheres, 1)
                    center = obstacles.spheres(j, 1:3);
                    radius = obstacles.spheres(j, 4);
                    safetyMargin = radius * safetyMarginRatio;
                    dist = norm(samplePoint - center);
                    
                    if dist < (radius + safetyMargin)
                        isValid = false;
                        collisionPoints = [collisionPoints; samplePoint];
                        return;  % 发现碰撞立即返回
                    end
                end
            end
        end
    end
end

end

