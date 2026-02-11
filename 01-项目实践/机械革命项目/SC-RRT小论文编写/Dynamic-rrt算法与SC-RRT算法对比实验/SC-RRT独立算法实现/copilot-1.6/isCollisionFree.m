function isFree = isCollisionFree(point1, point2, obstacles, m)
    % 功能：检测两点路径是否无碰撞（支持2D/3D，使用向量化操作提升性能）
    % 输入参数:
    %   point1    - 起始点 [x1, x2, ..., xm]
    %   point2    - 终止点 [x1, x2, ..., xm]
    %   obstacles - 障碍物结构体（circles: [x, y, radius], spheres: [x, y, z, radius]）
    %   m         - 维度（2或3）
    % 输出参数:
    %   isFree    - 是否无碰撞

    % 确保输入为行向量
    point1 = reshape(point1, 1, m);
    point2 = reshape(point2, 1, m);

    % 初始化返回值
    isFree = true;

    % 计算线段方向向量和长度
    direction = point2 - point1;
    segmentLength = norm(direction);
    if segmentLength <= eps
        return; % 两点重合，跳过检测
    end
    unitDirection = direction / segmentLength;

    % 使用精确解析几何检测（无需离散采样，性能更优）
    if m == 2
        if isfield(obstacles, 'circles') && ~isempty(obstacles.circles)
            centers = obstacles.circles(:, 1:2);
            radii = obstacles.circles(:, 3);
            
            % 向量化计算线段与所有圆的交点
            d = point1 - centers;  % n x 2
            a = dot(unitDirection, unitDirection);  % 标量
            b = 2 * sum(d .* unitDirection, 2);  % n x 1
            c = sum(d.^2, 2) - radii.^2;  % n x 1
            discriminant = b.^2 - 4 * a * c;  % n x 1
            
            % 找到有交点的障碍物
            hasIntersection = discriminant > -eps;
            if any(hasIntersection)
                sqrtDisc = sqrt(max(discriminant(hasIntersection), 0));
                t1 = (-b(hasIntersection) - sqrtDisc) / (2 * a);
                t2 = (-b(hasIntersection) + sqrtDisc) / (2 * a);
                
                % 检查交点是否在线段范围内
                if any((t1 >= -eps & t1 <= segmentLength + eps) | ...
                       (t2 >= -eps & t2 <= segmentLength + eps))
                    isFree = false;
                    return;
                end
            end
        end
    else
        if isfield(obstacles, 'spheres') && ~isempty(obstacles.spheres)
            centers = obstacles.spheres(:, 1:3);
            radii = obstacles.spheres(:, 4);
            
            % 向量化计算线段与所有球的交点
            d = point1 - centers;  % n x 3
            a = dot(unitDirection, unitDirection);  % 标量
            b = 2 * sum(d .* unitDirection, 2);  % n x 1
            c = sum(d.^2, 2) - radii.^2;  % n x 1
            discriminant = b.^2 - 4 * a * c;  % n x 1
            
            % 找到有交点的障碍物
            hasIntersection = discriminant > -eps;
            if any(hasIntersection)
                sqrtDisc = sqrt(max(discriminant(hasIntersection), 0));
                t1 = (-b(hasIntersection) - sqrtDisc) / (2 * a);
                t2 = (-b(hasIntersection) + sqrtDisc) / (2 * a);
                
                % 检查交点是否在线段范围内
                if any((t1 >= -eps & t1 <= segmentLength + eps) | ...
                       (t2 >= -eps & t2 <= segmentLength + eps))
                    isFree = false;
                    return;
                end
            end
        end
    end
end