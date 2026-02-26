function path_smooth = smoothPathSimple(path, obstacles, dim, num_smooth_iters)
% smoothPathSimple - 多阶段高质量路径平滑
%
% 四阶段策略：
%   1. 迭代加权平均 - 全局软化拐角 (多轮, 渐进式)
%   2. 曲率自适应平滑 - 对高曲率点额外平滑
%   3. PCHIP插值 - 生成C1连续平滑曲线 (高密度)
%   4. 插值后二次平滑 - 消除插值残余抖动
%
% 输入:
%   path              - 原始路径 [N×dim]
%   obstacles         - 障碍物列表
%   dim               - 空间维度 (2 或 3)
%   num_smooth_iters  - 加权平均总迭代次数 (默认: 10)
%
% 输出:
%   path_smooth       - 平滑后的路径 [M×dim]
%
% 特点:
%   - 多阶段渐进式平滑, 效果远优于单次平滑
%   - 曲率自适应: 对尖锐拐角施加更强平滑力
%   - 高密度PCHIP插值, 确保路径视觉平滑
%   - 每阶段碰撞验证, 保证安全性
%   - 保持起终点不变
%
% 作者: SC-RRT优化团队
% 日期: 2025-12-14 (增强版)

    if nargin < 4
        num_smooth_iters = 10;
    end
    
    n = size(path, 1);
    
    % 路径太短，无法平滑
    if n < 3
        path_smooth = path;
        return;
    end
    
    % 保存原始路径用于回退
    path_original = path;
    
    %% ====== 阶段1: 迭代加权平均平滑（渐进式） ======
    % 前半段使用较强平滑 [0.30, 0.40, 0.30]
    % 后半段使用较温和平滑 [0.20, 0.60, 0.20]
    % 保持起点和终点不变
    path_avg = path;
    
    for iter = 1:num_smooth_iters
        path_new = path_avg;
        n_curr = size(path_avg, 1);
        
        if n_curr <= 2
            break;
        end
        
        % 前半段迭代使用更强的平滑权重
        if iter <= ceil(num_smooth_iters / 2)
            w_side = 0.28;  % 较强平滑
            w_center = 1.0 - 2 * w_side;
        else
            w_side = 0.18;  % 温和平滑
            w_center = 1.0 - 2 * w_side;
        end
        
        % 向量化计算: 对所有中间点同时做加权平均
        path_new(2:end-1, :) = w_side * path_avg(1:end-2, :) + ...
                               w_center * path_avg(2:end-1, :) + ...
                               w_side * path_avg(3:end, :);
        
        % 强制保持起终点不变
        path_new(1, :) = path_original(1, :);
        path_new(end, :) = path_original(end, :);
        
        % 碰撞检测
        collision_free = true;
        for i = 1:size(path_new, 1)-1
            if ~isCollisionFree(path_new(i, :), path_new(i+1, :), obstacles, dim)
                collision_free = false;
                break;
            end
        end
        
        if collision_free
            path_avg = path_new;
        else
            % 本轮导致碰撞, 尝试更温和的权重
            w_side_mild = 0.10;
            w_center_mild = 0.80;
            path_mild = path_avg;
            path_mild(2:end-1, :) = w_side_mild * path_avg(1:end-2, :) + ...
                                    w_center_mild * path_avg(2:end-1, :) + ...
                                    w_side_mild * path_avg(3:end, :);
            path_mild(1, :) = path_original(1, :);
            path_mild(end, :) = path_original(end, :);
            
            mild_ok = true;
            for i = 1:size(path_mild, 1)-1
                if ~isCollisionFree(path_mild(i, :), path_mild(i+1, :), obstacles, dim)
                    mild_ok = false;
                    break;
                end
            end
            
            if mild_ok
                path_avg = path_mild;
            end
            % 如果温和平滑也碰撞, 停止本轮但不退出(继续下轮)
        end
    end
    
    %% ====== 阶段2: 曲率自适应平滑 ======
    % 检测高曲率点(尖锐拐角)并对其施加额外平滑
    for pass = 1:3
        n_curr = size(path_avg, 1);
        if n_curr < 5
            break;
        end
        
        path_curv = path_avg;
        modified = false;
        
        for i = 3:n_curr-2
            % 计算局部曲率(转角)
            v1 = path_avg(i, :) - path_avg(i-1, :);
            v2 = path_avg(i+1, :) - path_avg(i, :);
            len1 = norm(v1); len2 = norm(v2);
            if len1 < 1e-8 || len2 < 1e-8, continue; end
            
            cos_a = dot(v1, v2) / (len1 * len2);
            cos_a = max(-1, min(1, cos_a));
            angle_deg = acos(cos_a) * 180 / pi;
            
            % 对转角 > 15度的点施加5点平均
            if angle_deg > 15
                path_curv(i, :) = 0.10 * path_avg(i-2, :) + ...
                                  0.20 * path_avg(i-1, :) + ...
                                  0.40 * path_avg(i, :) + ...
                                  0.20 * path_avg(i+1, :) + ...
                                  0.10 * path_avg(i+2, :);
                modified = true;
            end
        end
        
        if ~modified
            break;
        end
        
        path_curv(1, :) = path_original(1, :);
        path_curv(end, :) = path_original(end, :);
        
        % 验证碰撞
        curv_ok = true;
        for i = 1:size(path_curv, 1)-1
            if ~isCollisionFree(path_curv(i, :), path_curv(i+1, :), obstacles, dim)
                curv_ok = false;
                break;
            end
        end
        
        if curv_ok
            path_avg = path_curv;
        else
            break;
        end
    end
    
    %% ====== 阶段3: 高密度PCHIP插值 ======
    try
        n_avg = size(path_avg, 1);
        
        if n_avg < 3
            path_smooth = path_avg;
            return;
        end
        
        % 计算累计弧长作为参数化变量
        diffs = diff(path_avg);
        seg_lens = sqrt(sum(diffs.^2, 2));
        t = [0; cumsum(seg_lens)];
        
        if t(end) < 1e-10
            path_smooth = path_avg;
            return;
        end
        
        t = t / t(end);  % 归一化到 [0, 1]
        
        % 处理重复参数值
        for i = 2:length(t)
            if t(i) <= t(i-1)
                t(i) = t(i-1) + 1e-10;
            end
        end
        
        % 高密度目标点数: 确保视觉平滑
        target_n = max(n_avg * 3, min(n_avg * 5, 200));
        t_fine = linspace(0, 1, target_n)';
        
        % PCHIP插值
        path_pchip = zeros(target_n, dim);
        for d = 1:dim
            path_pchip(:, d) = interp1(t, path_avg(:, d), t_fine, 'pchip');
        end
        
        % 强制起终点精确
        path_pchip(1, :) = path_original(1, :);
        path_pchip(end, :) = path_original(end, :);
        
        %% ====== 阶段4: 插值后二次平滑 ======
        % 对PCHIP插值结果再做轻量加权平均, 消除微小抖动
        for post_iter = 1:5
            path_post = path_pchip;
            n_p = size(path_pchip, 1);
            if n_p <= 4, break; end
            
            path_post(2:end-1, :) = 0.15 * path_pchip(1:end-2, :) + ...
                                    0.70 * path_pchip(2:end-1, :) + ...
                                    0.15 * path_pchip(3:end, :);
            path_post(1, :) = path_original(1, :);
            path_post(end, :) = path_original(end, :);
            
            post_ok = true;
            for i = 1:size(path_post, 1)-1
                if ~isCollisionFree(path_post(i, :), path_post(i+1, :), obstacles, dim)
                    post_ok = false;
                    break;
                end
            end
            
            if post_ok
                path_pchip = path_post;
            else
                break;
            end
        end
        
        % 碰撞检测验证
        collision_free = true;
        for i = 1:size(path_pchip, 1)-1
            if ~isCollisionFree(path_pchip(i, :), path_pchip(i+1, :), obstacles, dim)
                collision_free = false;
                break;
            end
        end
        
        if collision_free
            path_smooth = path_pchip;
        else
            % PCHIP路径碰撞，使用加权平均结果
            path_smooth = path_avg;
        end
        
    catch
        % 插值出错，使用加权平均结果
        path_smooth = path_avg;
    end
    
    %% ====== 最终验证 ======
    % 放宽长度约束: 平滑后路径可比原始略长(20%以内均可接受)
    len_original = sum(sqrt(sum(diff(path_original).^2, 2)));
    len_smooth = sum(sqrt(sum(diff(path_smooth).^2, 2)));
    
    if len_smooth > len_original * 1.20
        % 平滑后路径过长，回退到原始路径
        path_smooth = path_original;
    end
end
