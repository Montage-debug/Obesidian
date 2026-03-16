% =========================================================================
%          SC-RRT Bidirectional Path Planning - Main Program
% =========================================================================
% Algorithm: SC_RRT_Bidirectional (Dual-tree RRT with dual ellipsoid constraints)
% Core Innovations:
%   1. Non-symmetric dual ellipsoid sampling constraints
%   2. Dynamic meet point calculation using potential field
%   3. Pareto frontier optimization for node selection
%   4. Adaptive PID heuristic cost tuning
% =========================================================================
clear; clc; close all;
addpath(genpath(pwd));


% =================== 参数结构体集中管理 ===================
configParams = struct(...
    'sc_rrt_mode', 'adaptive', ...              % 算法模式: 'basic'|'pid'|'adaptive'
    'env_idx', 2, ...                           % 环境索引 1-4
    'enable_path_smoothing', true, ...          % 启用路径平滑
    'path_smoothing_points', 150, ...           % 平滑后路径点数
    'use_pareto_optimization', true, ...        % 启用Pareto前沿优化
    'pareto_update_interval', 4, ...            % Pareto更新间隔
    'pareto_non_optimal_prob', 0.5, ...         % 非Pareto节点选择概率
    'dual_ellipsoid_update_interval', 2, ...    % 椭球体更新间隔(整数)
    'dual_ellipsoid_smoothing', 0.9, ...        % 交汇点平滑因子
    'dual_ellipsoid_buffer', 2.0, ...           % 椭球体缓冲系数
    'visualize_dual_ellipsoid', true, ...       % 启用椭球体可视化
    'max_iterations', 10000, ...                % 最大迭代次数
    'enable_realtime_visualization', true, ... % 实时可视化(关闭更快)
    'visualization_interval', 2, ...            % 可视化更新间隔
    'enable_gif_recording', false, ...          % GIF录制
    'gif_frame_interval', 10, ...
    'gif_delay_time', 0.05 ...
);

%% =========================================================================
%  Environment Presets
%  Description: 4 pre-configured environments, select via env_idx
% =========================================================================
env_configs = {
    % Environment 1: 2D with fixed obstacles (reproducible)
    struct('dimension','2D','bounds',[0 1500 0 1500],...
           'startPoint',[400 400],'goalPoint',[1100 1100],...
           'numObstacles',225,'obstacleRadius',15,'seed',42,...
           'description','2D Fixed Obstacles'),
    
    % Environment 2: 2D with random obstacles (varies each run)
    struct('dimension','2D','bounds',[0 1500 0 1500],...
           'startPoint',[400 400],'goalPoint',[1100 1100],...
           'numObstacles',225,'obstacleRadius',15,'seed',NaN,...
           'description','2D Random Obstacles'),
    
    % Environment 3: 3D with fixed obstacles (reproducible)
    struct('dimension','3D','bounds',[0 1500 0 1500 0 1500],...
           'startPoint',[400 400 400],'goalPoint',[1100 1100 1100],...
           'numObstacles',1500,'obstacleRadius',18,'seed',123,...
           'description','3D Fixed Obstacles'),
    
    % Environment 4: 3D with random obstacles (varies each run)
    struct('dimension','3D','bounds',[0 1500 0 1500 0 1500],...
           'startPoint',[400 400 400],'goalPoint',[1100 1100 1100],...
           'numObstacles',1500,'obstacleRadius',18,'seed',NaN,...
           'description','3D Random Obstacles')
};

config = env_configs{configParams.env_idx};

% GIF 文件路径
if configParams.enable_gif_recording
    gif_filename = sprintf('SC_RRT_Bidirectional_%s_env%d_%s.gif',config.dimension,configParams.env_idx,datestr(now,'yyyymmdd_HHMMSS'));
    gif_save_path = fullfile(pwd,gif_filename);
else
    gif_save_path = '';
end

%% 环境初始化
dimension      = config.dimension;
bounds         = config.bounds;
startPoint     = config.startPoint;
goalPoint      = config.goalPoint;
numObstacles   = config.numObstacles;
obstacleRadius = config.obstacleRadius;

if ~isnan(config.seed), rng(config.seed); else, rng('shuffle'); end

obstacles = generateObstacles(dimension,bounds,numObstacles,obstacleRadius,startPoint,goalPoint);

%% 图形窗口初始化
fig = figure('Name',['SC-RRT Bidirectional - ' config.description],...
             'Position',[100 100 900 700],...
             'Renderer','opengl',...
             'DoubleBuffer','on',...
             'BackingStore','off');
setupPlot(fig,dimension,bounds,startPoint,goalPoint,obstacles);

% 优化渲染性能
set(fig, 'GraphicsSmoothing', 'on');
drawnow limitrate;

%% =========================================================================
%  SC-RRT Bidirectional 算法执行
% =========================================================================
tic;
fprintf('\n========== SC-RRT Bidirectional Path Planning ==========\n');
fprintf('Mode: %s\n', configParams.sc_rrt_mode);
fprintf('Environment: %s\n', config.description);
if configParams.enable_realtime_visualization
    fprintf('Visualization: ON (slower)\n');
else
    fprintf('Visualization: OFF (faster)\n');
end
fprintf('========================================================\n\n');


[treeA,treeB,path,success,frame_count] = SC_RRT_Bidirectional(...
    startPoint,goalPoint,bounds,obstacles,fig,gif_save_path,...
    configParams.gif_frame_interval,configParams.gif_delay_time,...
    'Mode',configParams.sc_rrt_mode,...
    'MaxIterations',configParams.max_iterations,...
    'UpdateInterval',configParams.dual_ellipsoid_update_interval,...
    'SmoothingFactor',configParams.dual_ellipsoid_smoothing,...
    'EllipsoidBuffer',configParams.dual_ellipsoid_buffer,...
    'VisualizeDualEllipsoid',configParams.visualize_dual_ellipsoid,...
    'UseParetoFrontier',configParams.use_pareto_optimization,...
    'EnableVisualization',configParams.enable_realtime_visualization,...
    'VisualizationInterval',configParams.visualization_interval);
tree = {treeA,treeB};
elapsedTime = toc;
%% =========================================================================
%  路径平滑与结果处理
% =========================================================================
if success && ~isempty(path)
    % 路径平滑（PCHIP方法）
    if configParams.enable_path_smoothing
        fprintf('\n[Path Smoothing] Method: PCHIP, Output Points: %d\n', configParams.path_smoothing_points);
        path_original = path;
        path = smoothPathPCHIP(path, configParams.path_smoothing_points);
        
        % ========== 碰撞检测验证 ==========
        m = length(startPoint);
        [isPathValid, collisionPts] = validatePathCollision(path, obstacles, m, 30);
        
        if ~isPathValid
            warning('[Path Smoothing] ⚠️ 平滑路径与障碍物碰撞！回退到原始路径');
            fprintf('[Path Smoothing] 碰撞点数量: %d\n', size(collisionPts, 1));
            path = path_original;
            len_smoothed = calculatePathLength(path);
        end
        
        % 比较平滑前后长度
        len_original = calculatePathLength(path_original);
        len_smoothed = calculatePathLength(path);
        fprintf('[Path Smoothing] Original Length: %.2f, Smoothed: %.2f, Change: %.2f%%\n', ...
            len_original, len_smoothed, (len_smoothed-len_original)/len_original*100);
        
        if isPathValid
            fprintf('[Path Smoothing] ✅ 平滑路径碰撞检测通过\n');
        end
    end
    
    %% ========== 路径质量评估 ==========
    fprintf('\n========== 路径质量评估 ==========\n');
    m = length(startPoint);
    pathMetrics = evaluatePathQuality(path, obstacles, m);
    
    fprintf('📏 路径长度: %.2f\n', pathMetrics.length);
    fprintf('📐 平滑度 (平均转角): %.4f rad (%.2f°)\n', pathMetrics.smoothness, rad2deg(pathMetrics.smoothness));
    fprintf('🔄 平均曲率: %.6f\n', pathMetrics.curvature);
    fprintf('⚡ 最大曲率: %.6f\n', pathMetrics.maxCurvature);
    fprintf('🛡️  安全裕度 (最小间隙): %.2f\n', pathMetrics.safetyMargin);
    fprintf('📊 转角标准差: %.4f\n', pathMetrics.turnAngleStd);
    fprintf('⭐ 安全评分: %.2f/100\n', pathMetrics.clearanceScore);
    fprintf('🏆 综合质量评分: %.2f/100\n', pathMetrics.qualityScore);
    
    % 质量等级评定
    if pathMetrics.qualityScore >= 80
        qualityGrade = '优秀 ⭐⭐⭐';
    elseif pathMetrics.qualityScore >= 60
        qualityGrade = '良好 ⭐⭐';
    elseif pathMetrics.qualityScore >= 40
        qualityGrade = '中等 ⭐';
    else
        qualityGrade = '较差';
    end
    fprintf('📝 质量等级: %s\n', qualityGrade);
    fprintf('===================================\n\n');
    % 可视化最终路径
    figure(fig);
    visualizeResult(fig,dimension,path);
    drawnow;
    % 保存结果
    saveResultWrapper('SC_RRT_Bidirectional',config,dimension,configParams.env_idx,...
              path,tree,elapsedTime,obstacles,fig,gif_save_path);
end

% 打印统计信息
printStatistics(success,path,tree,elapsedTime,frame_count);
fprintf('\n============= Planning Complete =============\n');