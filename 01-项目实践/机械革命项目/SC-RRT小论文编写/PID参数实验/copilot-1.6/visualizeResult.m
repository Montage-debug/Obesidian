function visualizeResult(fig, dimension, path)
    % 可视化结果，仅二维才调用此函数
    figure(fig);
    plotPath(fig, dimension, path);
    
    % 强制图像重绘
    drawnow limitrate;
end