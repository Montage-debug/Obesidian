%% 测试修正后的SC-RRT（可视化验证）
clear; clc; close all;

fprintf('=== SC-RRT路径可视化修正测试 ===\n\n');

%% 添加路径
addpath(genpath('sc_rrt'));

%% 生成2D测试环境
fprintf('生成2D环境...\n');
env = generate2DEnvironment([0 100 0 100], 20);

%% 运行SC-RRT
fprintf('运行SC-RRT算法...\n');
[path, tree, success, metrics] = SC_RRT_Basic(env, 1000);

%% 检查结果
if success
    fprintf('✅ 路径规划成功\n');
    fprintf('   路径节点数: %d\n', size(path, 1));
    fprintf('   树节点数: %d (可视化)\n', size(tree.vertices, 1));
    fprintf('   规划时间: %.3f秒\n', metrics.planning_time);
    fprintf('   路径长度: %.2f\n', metrics.path_length);
    
    % 可视化
    fprintf('\n正在生成可视化...\n');
    visualizeAndSaveRRTResult(env, path, tree, success, 'SC_RRT_Fixed', 'results');
    fprintf('✅ 可视化已保存到 results/ 目录\n');
else
    fprintf('❌ 路径规划失败\n');
end

fprintf('\n=== 测试完成 ===\n');
