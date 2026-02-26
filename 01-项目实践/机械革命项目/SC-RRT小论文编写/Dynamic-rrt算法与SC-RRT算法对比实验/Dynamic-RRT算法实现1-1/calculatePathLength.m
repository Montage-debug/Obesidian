function pathLength = calculatePathLength(path)
    % 计算路径的总长度（支持任意维度）
    % 输入: path - N×m的矩阵，每一行代表路径上一个点的[x1,x2,...,xm]坐标
    % 输出: pathLength - 路径的总长度

    if size(path, 1) < 2
        pathLength = 0; % 路径点数少于2，返回0
        return;
    end
    
    % 使用矢量化操作计算路径长度
    diffPath = diff(path); % 计算连续点之间的差分
    squaredDiff = sum(diffPath.^2, 2); % 计算平方和
    pathLength = sum(sqrt(squaredDiff)); % 计算欧氏距离并求和
end