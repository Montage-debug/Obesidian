function [path_smooth, success] = smoothPathWithFillets(path, obstacles, dim, fillet_radius, density)
% smoothPathWithFillets - 对路径拐点进行圆角处理
%
% 输入:
%   path          - 原始路径 [N×dim]
%   obstacles     - 障碍物列表 [M×(dim+1)]
%                   2D: [center_x, center_y, radius]
%                   3D: [center_x, center_y, center_z, radius]
%   dim           - 空间维度 (2 或 3)
%   fillet_radius - 圆角半径 (默认: 自动计算)
%   density       - 圆角采样密度，每个圆角生成的点数 (默认: 10)
%
% 输出:
%   path_smooth   - 圆角平滑后的路径 [M×dim]
%   success       - 是否成功 (未发生碰撞)
%
% 功能:
%   1. 识别路径中的拐点
%   2. 对每个拐点进行圆角过渡处理
%   3. 使用圆弧或贝塞尔曲线连接相邻路段
%   4. 对每个圆角进行碰撞检测
%   5. 如果发生碰撞，减小圆角半径或跳过该拐点
%
% 算法原理:
%   对于路径上的拐点P(i)，连接P(i-1)、P(i)、P(i+1)三个点：
%   1. 计算转角angle = acos(dot(v1, v2))，其中v1 = P(i)-P(i-1)，v2 = P(i+1)-P(i)
%   2. 如果angle接近180度(直线)，跳过
%   3. 计算圆角的起点和终点，距离P(i)为fillet_radius
%   4. 使用圆弧或贝塞尔曲线连接起点和终点
%   5. 对圆弧上的点进行碰撞检测
%
% 作者: SC-RRT优化团队
% 日期: 2025-12-13

    % 参数默认值
    if nargin < 4 || isempty(fillet_radius)
        % 自动计算圆角半径：取平均步长的30%
        segment_lengths = vecnorm(diff(path), 2, 2);
        fillet_radius = mean(segment_lengths) * 0.3;
    end
    
    if nargin < 5
        density = 20;  % 每个圆角生成20个点（增加采样密度以提高碰撞检测精度）
    end
    
    success = true;
    path_smooth = [];
    collision_count = 0;  % 记录碰撞次数
    
    n = size(path, 1);
    
    % 路径太短，无需处理
    if n < 3
        path_smooth = path;
        return;
    end
    
    % 添加起点
    path_smooth = path(1, :);
    
    % 遍历所有中间点，检测拐点并添加圆角
    for i = 2:n-1
        p_prev = path(i-1, :);
        p_curr = path(i, :);
        p_next = path(i+1, :);
        
        % 计算方向向量
        v1 = p_curr - p_prev;
        v2 = p_next - p_curr;
        
        len1 = norm(v1);
        len2 = norm(v2);
        
        % 归一化
        if len1 < 1e-6 || len2 < 1e-6
            % 点重合，跳过
            continue;
        end
        
        v1_norm = v1 / len1;
        v2_norm = v2 / len2;
        
        % 计算夹角
        cos_angle = dot(v1_norm, v2_norm);
        cos_angle = max(-1, min(1, cos_angle));  % 限制范围
        angle = acos(cos_angle);
        
        % 判断是否需要圆角处理
        % 1. 夹角过小(接近180度，即直线)，不处理
        % 2. 夹角过大(接近0度，即急转弯)，可能需要减小圆角半径
        angle_deg = rad2deg(angle);
        
        if angle_deg < 5  % 接近直线，跳过
            path_smooth = [path_smooth; p_curr];
            continue;
        end
        
        % 计算有效的圆角半径
        % 圆角半径不能超过两段路径长度的一半
        max_radius = min(len1, len2) * 0.45;  % 留一些余量
        effective_radius = min(fillet_radius, max_radius);
        
        if effective_radius < 1e-3
            % 半径太小，不处理
            path_smooth = [path_smooth; p_curr];
            continue;
        end
        
        % 计算圆角的起点和终点
        % 起点：从p_curr沿v1反向移动effective_radius距离
        % 终点：从p_curr沿v2正向移动effective_radius距离
        fillet_start = p_curr - effective_radius * v1_norm;
        fillet_end = p_curr + effective_radius * v2_norm;
        
        % 生成圆角曲线
        % 使用二次贝塞尔曲线：B(t) = (1-t)^2*P0 + 2t(1-t)*P1 + t^2*P2
        % P0 = fillet_start, P1 = p_curr (控制点), P2 = fillet_end
        fillet_points = [];
        
        for t = linspace(0, 1, density)
            % 二次贝塞尔曲线公式
            p_bezier = (1-t)^2 * fillet_start + ...
                       2*t*(1-t) * p_curr + ...
                       t^2 * fillet_end;
            fillet_points = [fillet_points; p_bezier];
        end
        
        % 对圆角曲线进行碰撞检测（增强版）
        fillet_valid = true;
        
        if ~isempty(obstacles)
            % 1. 检查圆角曲线上的所有采样点
            for j = 1:size(fillet_points, 1)
                if ~checkPointCollisionFree(fillet_points(j, :), obstacles, dim)
                    fillet_valid = false;
                    break;
                end
            end
            
            % 2. 额外检查连接段（从上一点到圆角起点）
            if fillet_valid && size(path_smooth, 1) > 0
                last_point = path_smooth(end, :);
                if ~checkSegmentCollisionFree(last_point, fillet_start, obstacles, dim, 5)
                    fillet_valid = false;
                end
            end
            
            % 3. 检查圆角曲线段之间的连接（高密度采样）
            if fillet_valid
                for j = 1:size(fillet_points, 1)-1
                    if ~checkSegmentCollisionFree(fillet_points(j, :), fillet_points(j+1, :), obstacles, dim, 5)
                        fillet_valid = false;
                        break;
                    end
                end
            end
        end
        
        % 如果圆角有效，添加圆角点；否则添加原拐点
        if fillet_valid
            % 保存当前路径状态，以便验证失败时回滚
            path_smooth_before = path_smooth;
            
            % 添加圆角起点前的直线段
            if size(path_smooth, 1) > 0
                % 检查是否需要插入直线段
                last_point = path_smooth(end, :);
                if norm(fillet_start - last_point) > 1e-6
                    path_smooth = [path_smooth; fillet_start];
                end
            end
            
            % 添加圆角曲线(去除首尾点以避免重复)
            path_smooth = [path_smooth; fillet_points(2:end-1, :)];
            
            % 添加圆角终点
            path_smooth = [path_smooth; fillet_end];
            
            % 二次验证：检查新添加的段是否真的无碰撞（高密度采样）
            verification_failed = false;
            if ~isempty(obstacles) && size(path_smooth, 1) > size(path_smooth_before, 1)
                % 检查新添加的所有段
                start_idx = max(1, size(path_smooth_before, 1));
                for idx = start_idx:size(path_smooth, 1)-1
                    if ~checkSegmentCollisionFree(path_smooth(idx, :), path_smooth(idx+1, :), obstacles, dim, 20)
                        verification_failed = true;
                        break;
                    end
                end
            end
            
            % 如果二次验证失败，回滚并使用原拐点
            if verification_failed
                path_smooth = path_smooth_before;
                path_smooth = [path_smooth; p_curr];
                success = false;
                collision_count = collision_count + 1;
            end
        else
            % 圆角碰撞，尝试逐步减小半径直到成功
            retry_success = false;
            % 增加更多尝试次数，使用更精细的递减比例
            reduction_factors = [0.85, 0.7, 0.55, 0.4, 0.25, 0.15, 0.08];
            
            for reduction_factor = reduction_factors
                reduced_radius = effective_radius * reduction_factor;
                
                % 最小半径阈值
                if reduced_radius < 0.5
                    break;
                end
                
                % 重新计算圆角
                fillet_start_retry = p_curr - reduced_radius * v1_norm;
                fillet_end_retry = p_curr + reduced_radius * v2_norm;
                
                % 增加采样密度以提高精度
                retry_density = max(density, 25);
                fillet_points_retry = [];
                for t = linspace(0, 1, retry_density)
                    p_bezier = (1-t)^2 * fillet_start_retry + ...
                               2*t*(1-t) * p_curr + ...
                               t^2 * fillet_end_retry;
                    fillet_points_retry = [fillet_points_retry; p_bezier];
                end
                
                % 碰撞检测（增强版，高密度采样）
                retry_valid = true;
                
                % 检查圆角点
                for j = 1:size(fillet_points_retry, 1)
                    if ~checkPointCollisionFree(fillet_points_retry(j, :), obstacles, dim)
                        retry_valid = false;
                        break;
                    end
                end
                
                % 检查连接段（增加采样密度）
                if retry_valid && size(path_smooth, 1) > 0
                    last_point = path_smooth(end, :);
                    if ~checkSegmentCollisionFree(last_point, fillet_start_retry, obstacles, dim, 15)
                        retry_valid = false;
                    end
                end
                
                % 检查圆角段内部连接（增加采样密度）
                if retry_valid
                    for j = 1:size(fillet_points_retry, 1)-1
                        if ~checkSegmentCollisionFree(fillet_points_retry(j, :), fillet_points_retry(j+1, :), obstacles, dim, 8)
                            retry_valid = false;
                            break;
                        end
                    end
                end
                
                if retry_valid
                    % 成功，使用减小后的圆角
                    if size(path_smooth, 1) > 0
                        last_point = path_smooth(end, :);
                        if norm(fillet_start_retry - last_point) > 1e-6
                            path_smooth = [path_smooth; fillet_start_retry];
                        end
                    end
                    path_smooth = [path_smooth; fillet_points_retry(2:end-1, :)];
                    path_smooth = [path_smooth; fillet_end_retry];
                    retry_success = true;
                    break;
                end
            end
            
            % 如果所有尝试都失败，使用原拐点
            if ~retry_success
                path_smooth = [path_smooth; p_curr];
                success = false;  % 标记有拐点无法圆角
                collision_count = collision_count + 1;
            end
        end
    end
    
    % 添加终点
    path_smooth = [path_smooth; path(end, :)];
    
    % 清理重复点
    path_smooth = removeConsecutiveDuplicates(path_smooth);
    
    % 最终全路径碰撞检测验证（严格模式，高密度采样）
    final_collision = false;
    collision_segment = -1;
    if ~isempty(obstacles)
        for i = 1:size(path_smooth, 1)-1
            if ~checkSegmentCollisionFree(path_smooth(i, :), path_smooth(i+1, :), obstacles, dim, 25)
                final_collision = true;
                collision_segment = i;
                break;  % 发现碰撞立即停止检查
            end
        end
    end
    
    % 如果最终验证发现碰撞，只回退到原始路径（已经尽力优化）
    if final_collision
        fprintf('    ⚠ 圆角路径在段%d处检测到碰撞，已保留原始路径以确保安全\n', collision_segment);
        path_smooth = path;  % 回退到原始路径
        success = false;
    elseif collision_count > 0
        fprintf('    ✓ 圆角平滑完成: %d个拐点已圆角，%d个保留原样（碰撞风险）\n', n-2-collision_count, collision_count);
    else
        fprintf('    ✓ 圆角平滑完成: 所有%d个拐点已成功圆角化\n', n-2);
    end
end


function is_free = checkSegmentCollisionFree(p1, p2, obstacles, dim, num_checks)
% 检查线段是否与障碍物碰撞
%
% 输入:
%   p1         - 线段起点 [1×dim]
%   p2         - 线段终点 [1×dim]
%   obstacles  - 障碍物列表
%   dim        - 维度
%   num_checks - 线段上检查的点数（默认10）
%
% 输出:
%   is_free    - 是否无碰撞

    if nargin < 5
        num_checks = 15;  % 默认采样15个点，提高检测精度
    end
    
    is_free = true;
    
    if isempty(obstacles)
        return;
    end
    
    % 在线段上均匀采样并检查每个点
    for alpha = linspace(0, 1, num_checks)
        p_check = p1 + alpha * (p2 - p1);
        if ~checkPointCollisionFree(p_check, obstacles, dim)
            is_free = false;
            return;
        end
    end
end


function is_free = checkPointCollisionFree(point, obstacles, dim)
% 检查单个点是否与障碍物碰撞
%
% 输入:
%   point      - 待检测点 [1×dim]
%   obstacles  - 障碍物列表
%   dim        - 维度
%
% 输出:
%   is_free    - 是否无碰撞

    is_free = true;
    
    if isempty(obstacles)
        return;
    end
    
    for k = 1:size(obstacles, 1)
        if dim == 2
            % obstacles(k,:) = [center_x, center_y, radius]
            dist = norm(point - obstacles(k, 1:2));
            % 留10%安全裕度（增强避障安全性）
            if dist < obstacles(k, 3) * 1.10
                is_free = false;
                return;
            end
        elseif dim == 3
            % obstacles(k,:) = [center_x, center_y, center_z, radius]
            dist = norm(point - obstacles(k, 1:3));
            % 留10%安全裕度
            if dist < obstacles(k, 4) * 1.10
                is_free = false;
                return;
            end
        end
    end
end


function path_clean = removeConsecutiveDuplicates(path)
% 移除路径中连续的重复点
%
% 输入:
%   path        - 原始路径 [N×dim]
%
% 输出:
%   path_clean  - 清理后的路径 [M×dim], M <= N

    if size(path, 1) <= 1
        path_clean = path;
        return;
    end
    
    path_clean = path(1, :);
    
    for i = 2:size(path, 1)
        % 如果当前点与上一个点不同，添加
        if norm(path(i, :) - path_clean(end, :)) > 1e-6
            path_clean = [path_clean; path(i, :)];
        end
    end
end
