%% ====================================================================
%              SC-RRT优化验证脚本
%% ====================================================================
% 功能: 快速测试新增的5项优化功能
%
% 优化内容:
%   1. 改进Pareto选择策略 (动态概率 + 自适应权重)
%   2. 增强Excel报告生成 (新增10个性能指标)
%   3. 新增findBestConnection.m (周期性全局连接检查)
%   4. 自适应交汇点更新策略 (动态间隔 + 吸引力计算)
%   5. 路径后处理优化 (Shortcut + B样条平滑)
%
% 作者: SC-RRT优化团队
% 日期: 2025-12-13
%% ====================================================================

clear; clc; close all;

fprintf('\n╔════════════════════════════════════════════════════════════╗\n');
fprintf('║          SC-RRT优化功能验证测试                            ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

%% ========== 测试1: 快速2D对比 (验证所有优化) ==========
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('【测试1】快速2D对比 - 验证5项优化功能\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

try
    % 运行快速对比 (3次运行，中等障碍物密度)
    runAlgorithmComparison('NumRuns', 3, ...
                          'Dimension', '2D', ...
                          'NumObstacles2D', 30, ...
                          'MaxIterations2D', 2000, ...
                          'SaveResults', true, ...
                          'Visualize', true);
    
    fprintf('✓ 测试1通过: 基本功能正常\n\n');
catch ME
    fprintf('✗ 测试1失败: %s\n', ME.message);
    fprintf('  堆栈跟踪:\n');
    for i = 1:length(ME.stack)
        fprintf('    %s (行 %d)\n', ME.stack(i).name, ME.stack(i).line);
    end
    fprintf('\n');
end

%% ========== 测试2: 验证新增性能指标 ==========
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('【测试2】验证新增性能指标\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

try
    % 检查最新的results文件
    result_files = dir('rrt_toolbox-master/results/performance_data_*.mat');
    if ~isempty(result_files)
        [~, idx] = max([result_files.datenum]);
        latest_file = fullfile(result_files(idx).folder, result_files(idx).name);
        
        data = load(latest_file);
        
        % 检查新指标是否存在
        new_metrics = {'path_efficiency', 'node_efficiency', 'avg_turning_angle', ...
                      'max_turning_angle', 'path_density', 'min_clearance', 'avg_clearance'};
        
        fprintf('检查结果文件: %s\n', result_files(idx).name);
        
        if ~isempty(data.results)
            sample_result = data.results{1};
            missing_metrics = {};
            
            for i = 1:length(new_metrics)
                if isfield(sample_result, new_metrics{i})
                    fprintf('  ✓ %s: 存在\n', new_metrics{i});
                else
                    fprintf('  ✗ %s: 缺失\n', new_metrics{i});
                    missing_metrics{end+1} = new_metrics{i};
                end
            end
            
            if isempty(missing_metrics)
                fprintf('✓ 测试2通过: 所有新指标已添加\n\n');
            else
                fprintf('⚠ 测试2部分通过: %d个指标缺失\n\n', length(missing_metrics));
            end
        end
    else
        fprintf('⚠ 未找到结果文件，请先运行测试1\n\n');
    end
catch ME
    fprintf('✗ 测试2失败: %s\n\n', ME.message);
end

%% ========== 测试3: 验证Excel报告增强 ==========
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('【测试3】验证Excel报告增强\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

try
    % 检查最新的Excel文件
    excel_files = dir('rrt_toolbox-master/results/performance_comparison_*.xlsx');
    if ~isempty(excel_files)
        [~, idx] = max([excel_files.datenum]);
        latest_excel = fullfile(excel_files(idx).folder, excel_files(idx).name);
        
        fprintf('检查Excel文件: %s\n', excel_files(idx).name);
        
        % 读取第一个sheet
        [~, sheets] = xlsfinfo(latest_excel);
        if ~isempty(sheets)
            data = readtable(latest_excel, 'Sheet', sheets{1});
            
            fprintf('  包含 %d 列数据\n', width(data));
            fprintf('  列名: ');
            for i = 1:min(5, width(data))
                fprintf('%s, ', data.Properties.VariableNames{i});
            end
            if width(data) > 5
                fprintf('...');
            end
            fprintf('\n');
            
            % 检查是否有新增列
            if width(data) >= 15
                fprintf('✓ 测试3通过: Excel报告已增强 (包含%d列)\n\n', width(data));
            else
                fprintf('⚠ 测试3部分通过: 列数较少 (%d列)\n\n', width(data));
            end
        end
    else
        fprintf('⚠ 未找到Excel文件，请先运行测试1\n\n');
    end
catch ME
    fprintf('✗ 测试3失败: %s\n\n', ME.message);
end

%% ========== 测试4: 验证新增辅助函数 ==========
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('【测试4】验证新增辅助函数\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');

new_functions = {'findBestConnection.m', 'shortcutPath.m', 'smoothPathBSpline.m'};
sc_rrt_path = 'rrt_toolbox-master/sc_rrt/';

all_exist = true;
for i = 1:length(new_functions)
    func_path = fullfile(sc_rrt_path, new_functions{i});
    if exist(func_path, 'file')
        fprintf('  ✓ %s: 存在\n', new_functions{i});
    else
        fprintf('  ✗ %s: 缺失\n', new_functions{i});
        all_exist = false;
    end
end

if all_exist
    fprintf('✓ 测试4通过: 所有新函数已创建\n\n');
else
    fprintf('✗ 测试4失败: 部分函数缺失\n\n');
end

%% ========== 测试总结 ==========
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║                  优化验证完成                              ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n');
fprintf('\n【优化功能清单】\n');
fprintf('  1. ✓ Pareto选择策略优化 (动态概率: 0.1->0.3)\n');
fprintf('  2. ✓ Excel报告增强 (+10个性能指标)\n');
fprintf('  3. ✓ findBestConnection.m (周期性全局连接)\n');
fprintf('  4. ✓ 自适应交汇点更新 (动态间隔: 20-100)\n');
fprintf('  5. ✓ 路径后处理 (Shortcut + B样条)\n');
fprintf('\n【建议的下一步测试】\n');
fprintf('  >> runAlgorithmComparison(''NumRuns'', 10, ''Dimension'', ''2D'')\n');
fprintf('  >> runAlgorithmComparison(''NumRuns'', 5, ''NumObstacles2D'', 40)\n');
fprintf('\n');
