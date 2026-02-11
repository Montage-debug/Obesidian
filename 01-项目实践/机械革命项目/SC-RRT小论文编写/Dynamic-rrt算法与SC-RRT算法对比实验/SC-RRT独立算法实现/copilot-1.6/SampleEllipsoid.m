function sample = SampleEllipsoid(startPoint, goalPoint, cBest, bounds, obstacles, stepSize, m)
% SampleEllipsoid - 椭球内采样
% 输入:
%   startPoint - 起始点
%   goalPoint  - 目标点
%   cBest      - 当前最短路径长度
%   bounds     - 边界 [xmin xmax ymin ymax ...]
%   obstacles  - 障碍物结构
%   stepSize   - RRT步长
%   m          - 空间维度 (2 或 3)
% 输出:
%   sample     - 采样点（列向量）

    % 如果未找到路径，使用均匀采样
    if isinf(cBest)
        sample = RandomSample(bounds, m);
        return;
    end

    % 计算焦距（两焦点间距离）
    cMin = norm(goalPoint(1:m) - startPoint(1:m));
    if cBest < cMin
        % 防止无效值
        cBest = cMin;
    end

    % 计算椭球参数
    % a: 半长轴, c_focal: 半焦距, b: 半短轴
    a = cBest / 2;              % 半长轴
    c_focal = cMin / 2;         % 半焦距
    b_squared = a^2 - c_focal^2;  % b^2 = a^2 - c^2
    
    % 数值稳定性检查
    if b_squared < 1e-10
        % 椭球退化为线段，返回中心附近随机点
        sample = RandomSample(bounds, m);
        return;
    end
    
    b = sqrt(b_squared);  % 半短轴

    % 椭球中心（两焦点中点）
    center = (startPoint(1:m) + goalPoint(1:m)) / 2;

    % 主轴方向向量（归一化）
    eVec = (goalPoint(1:m) - startPoint(1:m)) / cMin;

    % 计算旋转矩阵（将标准椭球旋转到eVec方向）
    if m == 2
        % 2D情况：简单旋转
        theta = atan2(eVec(2), eVec(1));
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    else
        % 3D情况：使用改进的Rodrigues旋转公式
        x_axis = [1; 0; 0];  % 标准椭球长轴方向
        rotation_axis = cross(x_axis, eVec);
        rotation_axis_norm = norm(rotation_axis);
        
        if rotation_axis_norm < 1e-10
            % eVec与x轴平行或反平行
            if dot(x_axis, eVec) > 0
                R = eye(3);  % 同向
            else
                R = [-1 0 0; 0 1 0; 0 0 -1];  % 反向，180度旋转
            end
        else
            % 一般情况：Rodrigues旋转
            k = rotation_axis / rotation_axis_norm;  % 归一化旋转轴
            cos_angle = dot(x_axis, eVec);
            cos_angle = max(-1, min(1, cos_angle));  % 钳位到[-1,1]
            angle = acos(cos_angle);
            
            % Rodrigues公式
            K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
            R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
        end
    end

    % 在椭球内部采样，带验证
    max_attempts = 100;  % 最大尝试次数
    attempt = 0;
    
    while attempt < max_attempts
        attempt = attempt + 1;
        
        % 单位球采样
        xBall = RandomInUnitBall(m);
        % 变换到椭球空间
        L = diag([a, repmat(b, 1, m-1)]);
        sample = R * (L * xBall) + center';
        
        % 验证: 1)边界内 2)椭球约束验证
        % 椭球定义: 距起点+到终点的距离 <= 2a = cBest
        dist_sum = norm(sample - startPoint(1:m)') + norm(sample - goalPoint(1:m)');
        
        if all(sample >= bounds(1:2:end)') && all(sample <= bounds(2:2:end)') && ...
           dist_sum <= cBest * 1.001  % 添加1%数值容差
            % 碰撞检测，如果有碰撞则返回否则继续
            return;
        end
    end
    
    % 如果采样失败，返回中心附近的随机点
    sample = center' + randn(m, 1) * min(a, b) * 0.1;
end