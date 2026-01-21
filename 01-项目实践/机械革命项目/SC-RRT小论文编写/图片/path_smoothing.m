% test_path_smoothing.m
% 测试路径平滑算法：B样条、圆弧过渡、混合方法
%
% 作者: Copilot-1.1
% 日期: 2025-12-04

clear; clc; close all;

fprintf('========== 路径平滑算法测试 ==========\n\n');

% ========== 创建测试路径 ==========
% 2D路径（带明显拐角）
rawPath2D = [
    0, 0;
    50, 10;
    100, 5;
    150, 30;
    200, 25;
    250, 60;
    300, 55;
    350, 90;
    400, 100
];

% 3D路径
rawPath3D = [
    0, 0, 0;
    50, 20, 10;
    100, 30, 30;
    150, 10, 50;
    200, 40, 70;
    250, 30, 90;
    300, 60, 100
];

% ========== 2D路径平滑对比 ==========
figure('Name', '2D路径平滑对比', 'Position', [100, 100, 1400, 800]);

% 原始路径
subplot(2, 3, 1);
plot(rawPath2D(:,1), rawPath2D(:,2), 'ro-', 'LineWidth', 2, 'MarkerSize', 8);
grid on; axis equal;
title('原始路径', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y');

% PCHIP平滑
subplot(2, 3, 2);
tic;
smoothPath_pchip = PathModule('smooth', rawPath2D, 'Method', 'pchip', 'NumPoints', 150);
time_pchip = toc;
plot(rawPath2D(:,1), rawPath2D(:,2), 'ro--', 'MarkerSize', 6, 'LineWidth', 1);
hold on;
plot(smoothPath_pchip(:,1), smoothPath_pchip(:,2), 'b-', 'LineWidth', 2);
grid on; axis equal;
title(sprintf('PCHIP平滑 (%.3fs)', time_pchip), 'FontSize', 14);
xlabel('X'); ylabel('Y');
legend('原始', 'PCHIP');

% B样条平滑
subplot(2, 3, 3);
tic;
smoothPath_bspline = PathModule('smooth', rawPath2D, 'Method', 'bspline', ...
    'NumPoints', 150, 'Smoothness', 0.7);
time_bspline = toc;
plot(rawPath2D(:,1), rawPath2D(:,2), 'ro--', 'MarkerSize', 6, 'LineWidth', 1);
hold on;
plot(smoothPath_bspline(:,1), smoothPath_bspline(:,2), 'g-', 'LineWidth', 2);
grid on; axis equal;
title(sprintf('B样条平滑 (%.3fs)', time_bspline), 'FontSize', 14);
xlabel('X'); ylabel('Y');
legend('原始', 'B样条');

% 圆弧过渡平滑
subplot(2, 3, 4);
tic;
smoothPath_arc = PathModule('smooth', rawPath2D, 'Method', 'arc', ...
    'NumPoints', 150, 'AngleThreshold', 15);
time_arc = toc;
plot(rawPath2D(:,1), rawPath2D(:,2), 'ro--', 'MarkerSize', 6, 'LineWidth', 1);
hold on;
plot(smoothPath_arc(:,1), smoothPath_arc(:,2), 'm-', 'LineWidth', 2);
grid on; axis equal;
title(sprintf('圆弧过渡平滑 (%.3fs)', time_arc), 'FontSize', 14);
xlabel('X'); ylabel('Y');
legend('原始', '圆弧过渡');

% 混合平滑
subplot(2, 3, 5);
tic;
smoothPath_hybrid = PathModule('smooth', rawPath2D, 'Method', 'hybrid', ...
    'NumPoints', 150, 'AngleThreshold', 20, 'Smoothness', 0.7);
time_hybrid = toc;
plot(rawPath2D(:,1), rawPath2D(:,2), 'ro--', 'MarkerSize', 6, 'LineWidth', 1);
hold on;
plot(smoothPath_hybrid(:,1), smoothPath_hybrid(:,2), 'c-', 'LineWidth', 2);
grid on; axis equal;
title(sprintf('混合平滑 (%.3fs)', time_hybrid), 'FontSize', 14);
xlabel('X'); ylabel('Y');
legend('原始', '混合');

% 总对比
subplot(2, 3, 6);
plot(rawPath2D(:,1), rawPath2D(:,2), 'ko-', 'MarkerSize', 8, 'LineWidth', 2);
hold on;
plot(smoothPath_pchip(:,1), smoothPath_pchip(:,2), 'b-', 'LineWidth', 1.5);
plot(smoothPath_bspline(:,1), smoothPath_bspline(:,2), 'g-', 'LineWidth', 1.5);
plot(smoothPath_arc(:,1), smoothPath_arc(:,2), 'm-', 'LineWidth', 1.5);
plot(smoothPath_hybrid(:,1), smoothPath_hybrid(:,2), 'c-', 'LineWidth', 2.5);
grid on; axis equal;
title('总对比', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y');
legend('原始', 'PCHIP', 'B样条', '圆弧', '混合', 'Location', 'best');

% ========== 3D路径平滑对比 ==========
figure('Name', '3D路径平滑对比', 'Position', [150, 150, 1400, 800]);

% 原始路径
subplot(2, 3, 1);
plot3(rawPath3D(:,1), rawPath3D(:,2), rawPath3D(:,3), 'ro-', 'LineWidth', 2, 'MarkerSize', 8);
grid on; axis equal;
title('原始3D路径', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
view(45, 30);

% PCHIP平滑
subplot(2, 3, 2);
smoothPath3D_pchip = PathModule('smooth', rawPath3D, 'Method', 'pchip', 'NumPoints', 150);
plot3(rawPath3D(:,1), rawPath3D(:,2), rawPath3D(:,3), 'ro--', 'MarkerSize', 6);
hold on;
plot3(smoothPath3D_pchip(:,1), smoothPath3D_pchip(:,2), smoothPath3D_pchip(:,3), 'b-', 'LineWidth', 2);
grid on; axis equal;
title('PCHIP平滑', 'FontSize', 14);
xlabel('X'); ylabel('Y'); zlabel('Z');
view(45, 30);
legend('原始', 'PCHIP');

% B样条平滑
subplot(2, 3, 3);
smoothPath3D_bspline = PathModule('smooth', rawPath3D, 'Method', 'bspline', ...
    'NumPoints', 150, 'Smoothness', 0.7);
plot3(rawPath3D(:,1), rawPath3D(:,2), rawPath3D(:,3), 'ro--', 'MarkerSize', 6);
hold on;
plot3(smoothPath3D_bspline(:,1), smoothPath3D_bspline(:,2), smoothPath3D_bspline(:,3), 'g-', 'LineWidth', 2);
grid on; axis equal;
title('B样条平滑', 'FontSize', 14);
xlabel('X'); ylabel('Y'); zlabel('Z');
view(45, 30);
legend('原始', 'B样条');

% 圆弧过渡平滑
subplot(2, 3, 4);
smoothPath3D_arc = PathModule('smooth', rawPath3D, 'Method', 'arc', ...
    'NumPoints', 150, 'AngleThreshold', 15);
plot3(rawPath3D(:,1), rawPath3D(:,2), rawPath3D(:,3), 'ro--', 'MarkerSize', 6);
hold on;
plot3(smoothPath3D_arc(:,1), smoothPath3D_arc(:,2), smoothPath3D_arc(:,3), 'm-', 'LineWidth', 2);
grid on; axis equal;
title('圆弧过渡平滑', 'FontSize', 14);
xlabel('X'); ylabel('Y'); zlabel('Z');
view(45, 30);
legend('原始', '圆弧过渡');

% 混合平滑
subplot(2, 3, 5);
smoothPath3D_hybrid = PathModule('smooth', rawPath3D, 'Method', 'hybrid', ...
    'NumPoints', 150, 'AngleThreshold', 20, 'Smoothness', 0.7);
plot3(rawPath3D(:,1), rawPath3D(:,2), rawPath3D(:,3), 'ro--', 'MarkerSize', 6);
hold on;
plot3(smoothPath3D_hybrid(:,1), smoothPath3D_hybrid(:,2), smoothPath3D_hybrid(:,3), 'c-', 'LineWidth', 2);
grid on; axis equal;
title('混合平滑', 'FontSize', 14);
xlabel('X'); ylabel('Y'); zlabel('Z');
view(45, 30);
legend('原始', '混合');

% 所有方法叠加
subplot(2, 3, 6);
plot3(rawPath3D(:,1), rawPath3D(:,2), rawPath3D(:,3), 'ko-', 'MarkerSize', 8, 'LineWidth', 2);
hold on;
plot3(smoothPath3D_pchip(:,1), smoothPath3D_pchip(:,2), smoothPath3D_pchip(:,3), 'b-', 'LineWidth', 1.5);
plot3(smoothPath3D_bspline(:,1), smoothPath3D_bspline(:,2), smoothPath3D_bspline(:,3), 'g-', 'LineWidth', 1.5);
plot3(smoothPath3D_arc(:,1), smoothPath3D_arc(:,2), smoothPath3D_arc(:,3), 'm-', 'LineWidth', 1.5);
plot3(smoothPath3D_hybrid(:,1), smoothPath3D_hybrid(:,2), smoothPath3D_hybrid(:,3), 'c-', 'LineWidth', 2.5);
grid on; axis equal;
title('总对比', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X'); ylabel('Y'); zlabel('Z');
view(45, 30);
legend('原始', 'PCHIP', 'B样条', '圆弧', '混合', 'Location', 'best');

% ========== 性能对比 ==========
fprintf('\n========== 2D路径平滑性能对比 ==========\n');
fprintf('方法          计算时间(s)  路径长度\n');
fprintf('-------------------------------------------\n');
fprintf('PCHIP:      %.6f    %.2f\n', time_pchip, PathModule('length', smoothPath_pchip));
fprintf('B样条:      %.6f    %.2f\n', time_bspline, PathModule('length', smoothPath_bspline));
fprintf('圆弧过渡:   %.6f    %.2f\n', time_arc, PathModule('length', smoothPath_arc));
fprintf('混合方法:   %.6f    %.2f\n', time_hybrid, PathModule('length', smoothPath_hybrid));
fprintf('原始路径:   -         %.2f\n', PathModule('length', rawPath2D));
fprintf('===========================================\n\n');
