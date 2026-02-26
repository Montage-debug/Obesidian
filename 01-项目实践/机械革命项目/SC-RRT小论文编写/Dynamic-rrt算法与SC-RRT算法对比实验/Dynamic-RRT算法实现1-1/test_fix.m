% =========================================================================
%     快速修复验证 - 立即测试
% =========================================================================
% 用途: 快速验证SC-RRT参数修复是否成功
% 只测试一个SC-RRT算法，立即反馈
% =========================================================================

clear; clc; close all;

fprintf('\n');
fprintf('========================================\n');
fprintf('快速修复验证测试\n');
fprintf('========================================\n\n');

%% 配置
config = struct();
config.dimension = '2D';
config.bounds = [0 1500 0 1500];
config.startPoint = [400 400];
config.goalPoint = [1100 1100];
config.numObstacles = 225;
config.obstacleRadius = 15;
config.seed = 42;

fprintf('测试环境: 2D, 225障碍物\n\n');

%% 生成障碍物
fprintf('[1/3] 生成障碍物...\n');
sc_rrt_path = '../SC-RRT独立算法实现/copilot-1.6';
if exist(fullfile(sc_rrt_path, 'generateObstacles.m'), 'file')
    addpath(sc_rrt_path);
    rng(config.seed);
    obstacles = generateObstacles(config.dimension, ...
                                 config.bounds, ...
                                 config.numObstacles, ...
                                 config.obstacleRadius, ...
                                 config.startPoint, ...
                                 config.goalPoint);
    fprintf('  ✓ 完成 (%d个)\n\n', size(obstacles.circles, 1));
else
    fprintf('  ✗ 找不到SC-RRT路径\n');
    return;
end

%% 测试SC-RRT Adaptive
fprintf('[2/3] 测试 SC-RRT Adaptive (adaptive模式)...\n');

fig_temp = figure('Visible', 'off');

try
    tic;
    [treeA, treeB, path, success, ~, metrics] = SC_RRT_Bidirectional(...
        config.startPoint, ...
        config.goalPoint, ...
        config.bounds, ...
        obstacles, ...
        fig_temp, '', 0, 0, ...
        'Mode', 'adaptive', ...
        'MaxIterations', 5000, ...
        'UseParetoFrontier', true, ...
        'EnableVisualization', false);
    elapsed_time = toc;
    
    close(fig_temp);
    
    if success && ~isempty(path)
        fprintf('  ✓ 成功!\n');
        fprintf('    时间: %.4f 秒\n', elapsed_time);
        
        % 处理字段名
        if isfield(metrics, 'pathLength')
            path_len = metrics.pathLength;
        elseif isfield(metrics, 'finalPathLength')
            path_len = metrics.finalPathLength;
        else
            path_len = calculatePathLength(path);
        end
        
        fprintf('    路径长度: %.2f mm\n', path_len);
        fprintf('    节点数: %d + %d = %d\n', treeA.count, treeB.count, treeA.count+treeB.count);
        fprintf('\n');
        
        fprintf('========================================\n');
        fprintf('✅ 修复成功! SC-RRT可以正常运行\n');
        fprintf('========================================\n\n');
        
        fprintf('现在可以运行完整实验:\n');
        fprintf('  >> single_run_comparison\n\n');
        
    else
        fprintf('  ✗ 失败 (未找到路径，但参数正确)\n\n');
        fprintf('========================================\n');
        fprintf('⚠️ 参数修复成功，但此次未找到路径\n');
        fprintf('   这可能是随机性导致，请重新运行\n');
        fprintf('========================================\n\n');
    end
    
catch ME
    close(fig_temp);
    fprintf('  ✗ 错误: %s\n\n', ME.message);
    
    if contains(ME.message, 'Mode')
        fprintf('========================================\n');
        fprintf('❌ 仍有Mode参数问题\n');
        fprintf('========================================\n');
        fprintf('请检查SC-RRT版本是否支持以下模式:\n');
        fprintf('  - ''basic''\n');
        fprintf('  - ''pid''\n');
        fprintf('  - ''adaptive''\n\n');
        fprintf('如果错误持续，请提供SC-RRT的完整错误信息\n\n');
    else
        fprintf('========================================\n');
        fprintf('❌ 其他错误\n');
        fprintf('========================================\n');
        if ~isempty(ME.stack)
            fprintf('位置: %s (Line %d)\n\n', ME.stack(1).name, ME.stack(1).line);
        end
    end
end

%% 辅助函数
function length = calculatePathLength(path)
    if size(path, 1) < 2
        length = 0;
        return;
    end
    length = 0;
    for i = 2:size(path, 1)
        length = length + norm(path(i,:) - path(i-1,:));
    end
end
