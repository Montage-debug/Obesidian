function path_smooth = smoothPathBSpline(path, smoothing_factor)
% smoothPathBSpline - B样条曲线平滑
%
% 输入:
%   path             - 原始路径 [N×dim]
%   smoothing_factor - 平滑因子 (0-1, 默认: 0.3)
%                      0 = 严格插值
%                      1 = 最平滑
%
% 输出:
%   path_smooth      - 平滑后的路径 [M×dim]
%
% 功能:
%   使用B样条曲线对路径进行平滑处理
%
% 依赖:
%   需要MATLAB的Curve Fitting Toolbox (csaps函数)
%
% 作者: SC-RRT优化团队
% 日期: 2025-12-13

    if nargin < 2
        smoothing_factor = 0.3;
    end
    
    n = size(path, 1);
    
    % 如果路径太短，直接返回
    if n < 4
        path_smooth = path;
        return;
    end
    
    % 检查csaps是否可用
    if ~exist('csaps', 'file')
        warning('csaps函数不可用，需要Curve Fitting Toolbox。返回原始路径。');
        path_smooth = path;
        return;
    end
    
    try
        % 参数化
        t = linspace(0, 1, n)';
        
        % 为每个维度拟合B样条
        dim = size(path, 2);
        t_fine = linspace(0, 1, n*2)';  % 加密采样
        path_smooth = zeros(length(t_fine), dim);
        
        for d = 1:dim
            % csaps平滑样条拟合
            pp = csaps(t, path(:,d), smoothing_factor);
            path_smooth(:,d) = ppval(pp, t_fine);
        end
    catch ME
        warning('B样条平滑失败: %s。返回原始路径。', ME.message);
        path_smooth = path;
    end
end
