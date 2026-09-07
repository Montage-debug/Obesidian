function visualizeAndSaveRRTResult(env, path, tree, success, algorithm_name, save_dir, metrics)
% visualizeAndSaveRRTResult - 可视化并保存RRT规划结果
%
% 输入:
%   env: 环境结构体
%   path: 规划路径
%   tree: 树结构体
%   success: 是否成功
%   algorithm_name: 算法名称 (默认: 'RRT')
%   save_dir: 保存目录 (默认: 'results')
%   metrics: 性能指标结构体 (可选, 包含planning_time, path_length, smoothness等)
%
% 功能:
%   - 弹窗显示规划结果 (标题包含 时间/路径长度/节点数/平滑度)
%   - 自动保存图片到本地
%   - 生成时间戳文件名

%% 参数处理
if nargin < 5, algorithm_name = 'RRT'; end
if nargin < 6, save_dir = 'results'; end
if nargin < 7, metrics = struct(); end

% 创建保存目录
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

% 生成时间戳
timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');

if ~success
    fprintf('⚠ 规划失败,无法可视化\n');
    return;
end

%% 计算统计信息
path_length = 0;
for i = 1:size(path, 1)-1
    path_length = path_length + norm(path(i+1, :) - path(i, :));
end
num_nodes = size(tree.vertices, 1);
num_path_nodes = size(path, 1);

%% 可视化
is_2d = strcmp(env.dimension, '2D');

if is_2d
    % 2D可视化
    fig = figure('Name', sprintf('%s 路径规划结果 - 2D', algorithm_name), ...
                 'Position', [100 100 900 700], 'Color', 'w');
    hold on; axis equal; grid on;
    
    xlim([env.bounds(1), env.bounds(2)]);
    ylim([env.bounds(3), env.bounds(4)]);
    
    % 绘制障碍物
    for i = 1:env.num
        x = env.obstacles(i, 1);
        y = env.obstacles(i, 2);
        r = env.obstacles(i, 3);
        rectangle('Position', [x-r, y-r, 2*r, 2*r], ...
                 'Curvature', [1 1], ...
                 'FaceColor', [0.2 0.2 0.2], ...
                 'EdgeColor', [0.1 0.1 0.1], ...
                 'LineWidth', 0.5);
    end
    
    % 绘制搜索树（淡蓝色细线）
    % 注意：对于SC-RRT，tree已经是最终路径的线性结构
    has_tree_edges = false;
    for i = 2:size(tree.vertices, 1)
        parent_idx = tree.parent(i);
        if parent_idx > 0 && parent_idx < i  % 确保是有效的父节点关系
            if ~has_tree_edges
                % 第一条边，用于图例
                plot([tree.vertices(parent_idx, 1), tree.vertices(i, 1)], ...
                     [tree.vertices(parent_idx, 2), tree.vertices(i, 2)], ...
                     'Color', [0.7 0.9 1.0], 'LineWidth', 0.5);
                has_tree_edges = true;
            else
                % 后续边，不加入图例
                plot([tree.vertices(parent_idx, 1), tree.vertices(i, 1)], ...
                     [tree.vertices(parent_idx, 2), tree.vertices(i, 2)], ...
                     'Color', [0.7 0.9 1.0], 'LineWidth', 0.5, 'HandleVisibility', 'off');
            end
        end
    end
    
    % 绘制规划路径(粗红线，在搜索树之上)
    plot(path(:, 1), path(:, 2), 'r-', 'LineWidth', 3, 'DisplayName', '规划路径');
    plot(path(:, 1), path(:, 2), 'ro', 'MarkerSize', 4, 'MarkerFaceColor', 'r', ...
         'HandleVisibility', 'off');
    
    % 绘制起点和终点
    plot(env.start_point(1), env.start_point(2), 'go', ...
         'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
    plot(env.goal_point(1), env.goal_point(2), 'rs', ...
         'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 2);
    
    % 添加标签
    xlabel('X (m)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Y (m)', 'FontSize', 12, 'FontWeight', 'bold');
    
    % 标题包含统计信息 (类似single_run_comparison样式)
    if isfield(metrics, 'planning_time')
        plan_time = metrics.planning_time;
    else
        plan_time = NaN;
    end
    if isfield(metrics, 'smoothness') && ~isnan(metrics.smoothness)
        smooth_val = metrics.smoothness;
    else
        smooth_val = NaN;
    end
    title_str = sprintf('%s\nTime:%.3fs | Length:%.1fmm | Nodes:%d | Smoothness:%.4f', ...
                       algorithm_name, plan_time, path_length, num_nodes, smooth_val);
    title(title_str, 'FontSize', 13, 'FontWeight', 'bold');
    
    legend('障碍物', '搜索树', '规划路径', '起点', '终点', ...
           'Location', 'best', 'FontSize', 10);
    
    % 添加文本信息框
    info_text = sprintf('环境: %d×%d\n障碍物: %d个\n起点: [%.1f, %.1f]\n终点: [%.1f, %.1f]', ...
                       env.bounds(2)-env.bounds(1), env.bounds(4)-env.bounds(3), ...
                       env.num, env.start_point(1), env.start_point(2), ...
                       env.goal_point(1), env.goal_point(2));
    annotation('textbox', [0.02 0.02 0.2 0.15], 'String', info_text, ...
              'FitBoxToText', 'on', 'BackgroundColor', 'w', ...
              'EdgeColor', 'k', 'FontSize', 9);
    
else
    % 3D可视化
    fig = figure('Name', sprintf('%s 路径规划结果 - 3D', algorithm_name), ...
                 'Position', [100 50 1000 800], 'Color', 'w');
    hold on; axis equal; grid on;
    view(45, 30);
    
    xlim([env.bounds(1), env.bounds(2)]);
    ylim([env.bounds(3), env.bounds(4)]);
    zlim([env.bounds(5), env.bounds(6)]);
    
    % 绘制障碍物(球体)
    [X, Y, Z] = sphere(20);
    for i = 1:env.num
        cx = env.obstacles(i, 1);
        cy = env.obstacles(i, 2);
        cz = env.obstacles(i, 3);
        r = env.obstacles(i, 4);
        
        surf(X*r + cx, Y*r + cy, Z*r + cz, ...
             'FaceColor', [0 0 0], ...
             'EdgeColor', 'none', ...
             'FaceAlpha', 0.7);
    end
    
    % 绘制搜索树(降采样)
    sample_rate = max(1, floor(size(tree.vertices, 1) / 300));
    for i = 2:sample_rate:size(tree.vertices, 1)
        parent_idx = tree.parent(i);
        if parent_idx > 0  % 跳过根节点
            plot3([tree.vertices(parent_idx, 1), tree.vertices(i, 1)], ...
                  [tree.vertices(parent_idx, 2), tree.vertices(i, 2)], ...
                  [tree.vertices(parent_idx, 3), tree.vertices(i, 3)], ...
                  'Color', [0.7 0.9 1.0], 'LineWidth', 0.5);
        end
    end
    
    % 绘制规划路径
    plot3(path(:, 1), path(:, 2), path(:, 3), ...
          'r-', 'LineWidth', 4);
    plot3(path(:, 1), path(:, 2), path(:, 3), ...
          'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
    
    % 绘制起点和终点
    plot3(env.start_point(1), env.start_point(2), env.start_point(3), ...
          'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2);
    plot3(env.goal_point(1), env.goal_point(2), env.goal_point(3), ...
          'rs', 'MarkerSize', 15, 'MarkerFaceColor', 'r', 'LineWidth', 2);
    
    % 标签
    xlabel('X (m)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Y (m)', 'FontSize', 12, 'FontWeight', 'bold');
    zlabel('Z (m)', 'FontSize', 12, 'FontWeight', 'bold');
    
    % 标题 (类似single_run_comparison样式)
    if isfield(metrics, 'planning_time')
        plan_time = metrics.planning_time;
    else
        plan_time = NaN;
    end
    if isfield(metrics, 'smoothness') && ~isnan(metrics.smoothness)
        smooth_val = metrics.smoothness;
    else
        smooth_val = NaN;
    end
    title_str = sprintf('%s\nTime:%.3fs | Length:%.1fmm | Nodes:%d | Smoothness:%.4f', ...
                       algorithm_name, plan_time, path_length, num_nodes, smooth_val);
    title(title_str, 'FontSize', 13, 'FontWeight', 'bold');
    
    % 光照
    lighting gouraud;
    camlight('headlight');
    
    % 添加文本信息
    info_text = sprintf('环境: %d×%d×%d\n障碍物: %d个\n起点: [%.1f, %.1f, %.1f]\n终点: [%.1f, %.1f, %.1f]', ...
                       env.bounds(2)-env.bounds(1), env.bounds(4)-env.bounds(3), ...
                       env.bounds(6)-env.bounds(5), env.num, ...
                       env.start_point(1), env.start_point(2), env.start_point(3), ...
                       env.goal_point(1), env.goal_point(2), env.goal_point(3));
    annotation('textbox', [0.02 0.02 0.2 0.15], 'String', info_text, ...
              'FitBoxToText', 'on', 'BackgroundColor', 'w', ...
              'EdgeColor', 'k', 'FontSize', 9);
end

%% 保存图片
% 生成文件名
if is_2d
    filename = sprintf('%s/%s_2D_%s.png', save_dir, algorithm_name, timestamp);
else
    filename = sprintf('%s/%s_3D_%s.png', save_dir, algorithm_name, timestamp);
end

% 保存高分辨率图片
saveas(fig, filename);
print(fig, strrep(filename, '.png', '_hires.png'), '-dpng', '-r300');  % 高分辨率版本

fprintf('✓ 图片已保存:\n');
fprintf('  标准版本: %s\n', filename);
fprintf('  高分辨率: %s\n', strrep(filename, '.png', '_hires.png'));

%% 保存数据
data_filename = sprintf('%s/%s_%s_%s_data.mat', save_dir, algorithm_name, ...
                       iif(is_2d, '2D', '3D'), timestamp);
save(data_filename, 'env', 'path', 'tree', 'success', 'algorithm_name', ...
     'path_length', 'num_nodes', 'num_path_nodes');
fprintf('  数据文件: %s\n', data_filename);

fprintf('\n');

end

%% 辅助函数
function result = iif(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end
