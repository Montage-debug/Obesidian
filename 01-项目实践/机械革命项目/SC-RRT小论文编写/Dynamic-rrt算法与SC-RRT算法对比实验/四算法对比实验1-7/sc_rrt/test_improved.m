%% 简单测试SC-RRT基础功能
clear; clc; close all;

%% 2D测试
disp('=== 2D测试 ===');

% 创建2D环境结构体
env = struct();
env.dimension = 2;
env.bounds = [-5, 15; -5, 15];
env.start = [0, 0];
env.goal = [10, 10];
env.obstacles = [];
env.num_obstacles = 0;

try
    [path, ~, success] = SC_RRT_Basic(env, 500);
    if ~success || isempty(path)
        disp('❌ 2D测试失败: 未找到路径');
    else
        disp(['✅ 2D测试成功: 找到路径, 长度=' num2str(size(path,1))]);
    end
catch ME
    disp(['❌ 2D测试错误: ' ME.message]);
    if ~isempty(ME.stack)
        disp(['   位置: ' ME.stack(1).name ' 第' num2str(ME.stack(1).line) '行']);
    end
end

%% 3D测试
disp(' ');
disp('=== 3D测试 ===');

% 创建3D环境结构体
env = struct();
env.dimension = 3;
env.bounds = [-5, 15; -5, 15; -5, 15];
env.start = [0, 0, 0];
env.goal = [10, 10, 10];
env.obstacles = [];
env.num_obstacles = 0;

try
    [path, ~, success] = SC_RRT_Basic(env, 500);
    if ~success || isempty(path)
        disp('❌ 3D测试失败: 未找到路径');
    else
        disp(['✅ 3D测试成功: 找到路径, 长度=' num2str(size(path,1))]);
    end
catch ME
    disp(['❌ 3D测试错误: ' ME.message]);
    if ~isempty(ME.stack)
        disp(['   位置: ' ME.stack(1).name ' 第' num2str(ME.stack(1).line) '行']);
    end
end

disp(' ');
disp('=== 测试完成 ===');
