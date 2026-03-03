%% 简单测试SC-RRT基础功能
clear; clc; close all;

% 添加路径
addpath(genpath('sc_rrt'));

%% 2D测试
disp('=== 2D测试 ===');
start_point = [0, 0];
goal_point = [10, 10];
bounds = [-5, 15; -5, 15];
obstacles = {};

opts = struct();
opts.max_iter = 500;
opts.step_size = 0.5;
opts.goal_sample_rate = 0.1;
opts.vis_interval = 0;  % 关闭可视化

try
    [path, stats] = SC_RRT_Basic(start_point, goal_point, bounds, obstacles, opts);
    if isempty(path)
        disp('❌ 2D测试失败: 未找到路径');
    else
        disp(['✅ 2D测试成功: 找到路径, 长度=' num2str(size(path,1))]);
        disp(['   迭代次数: ' num2str(stats.iterations)]);
        disp(['   计算时间: ' num2str(stats.time, '%.3f') 's']);
    end
catch ME
    disp(['❌ 2D测试错误: ' ME.message]);
    disp(['   位置: ' ME.stack(1).name ' 第' num2str(ME.stack(1).line) '行']);
end

%% 3D测试
disp(' ');
disp('=== 3D测试 ===');
start_point = [0, 0, 0];
goal_point = [10, 10, 10];
bounds = [-5, 15; -5, 15; -5, 15];
obstacles = {};

opts.max_iter = 500;

try
    [path, stats] = SC_RRT_Basic(start_point, goal_point, bounds, obstacles, opts);
    if isempty(path)
        disp('❌ 3D测试失败: 未找到路径');
    else
        disp(['✅ 3D测试成功: 找到路径, 长度=' num2str(size(path,1))]);
        disp(['   迭代次数: ' num2str(stats.iterations)]);
        disp(['   计算时间: ' num2str(stats.time, '%.3f') 's']);
    end
catch ME
    disp(['❌ 3D测试错误: ' ME.message]);
    disp(['   位置: ' ME.stack(1).name ' 第' num2str(ME.stack(1).line) '行']);
end

disp(' ');
disp('=== 测试完成 ===');
