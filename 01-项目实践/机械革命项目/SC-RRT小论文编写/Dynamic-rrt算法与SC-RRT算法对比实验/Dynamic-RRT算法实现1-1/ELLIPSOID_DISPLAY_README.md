# SC-RRT 椭球体动态显示功能集成

## 概述

本文件提供了将 SC-RRT 算法的**动态椭球体显示**功能集成到 Dynamic-RRT 对比实验框架中的完整方案。

椭球体动态显示允许用户在规划过程中实时观察 SC-RRT 算法如何使用双椭球采样约束来引导搜索树的扩展。

## 新增文件说明

### 1. **plotEllipsoid.m** - 椭球体绘制核心模块
```matlab
function h = plotEllipsoid(focus1, focus2, cBest, m, color)
```
**功能**：在 2D/3D 空间中绘制椭圆或椭球面
- **输入参数**：
  - `focus1`, `focus2`：椭球体的两个焦点
  - `cBest`：椭球约束参数（焦点距离和）
  - `m`：维度（2 或 3）
  - `color`：RGB 颜色值
- **输出**：图形对象句柄

**使用示例**：
```matlab
% 绘制从起点到交汇点的椭圆（红色）
h1 = plotEllipsoid(startPoint, meetPoint, cBestA, 2, [0.8 0.3 0.3]);

% 绘制从交汇点到终点的椭圆（蓝色）
h2 = plotEllipsoid(meetPoint, goalPoint, cBestB, 2, [0.3 0.3 0.8]);
```

### 2. **calculateDualEllipsoidParams.m** - 双椭球参数计算
```matlab
function [c_best_A, c_best_B, c_min_A, c_min_B] = ...
    calculateDualEllipsoidParams(treeA, treeB, startPoint, goalPoint, meetPoint, m, buffer)
```
**功能**：计算双向 RRT 树的椭球约束参数
- **输入**：
  - `treeA`, `treeB`：起点树和终点树
  - `startPoint`, `goalPoint`：规划空间的起终点
  - `meetPoint`：动态交汇点
  - `m`：维度
  - `buffer`：缓冲系数（默认 1.2）
- **输出**：
  - `c_best_A`, `c_best_B`：两个椭球体的半长轴
  - `c_min_A`, `c_min_B`：两个椭球体的焦距

**数学原理**：
- 椭球体 A 的焦点：`(startPoint, meetPoint)`
- 椭球体 B 的焦点：`(meetPoint, goalPoint)`
- 椭球参数由树中节点的最优代价决定

### 3. **calculatePotentialMeetPoint.m** - 动态交汇点计算
```matlab
function [meetPoint, centroidA, centroidB] = ...
    calculatePotentialMeetPoint(treeA, treeB, startPoint, goalPoint, m)
```
**功能**：使用势场法计算动态交汇点
- 从优质节点集合计算树的有效质心
- 根据树的大小进行加权平均
- 确保交汇点在规划空间边界内

### 4. **SC_RRT_with_Ellipsoid_Display.m** - 完整集成模块
```matlab
function [tree, path, success, metrics, fig_ellipsoid] = ...
    SC_RRT_with_Ellipsoid_Display(startPoint, goalPoint, bounds, obstacles, varargin)
```
**功能**：SC-RRT 算法包装，支持椭球体实时显示
- 创建独立的椭球体显示窗口
- 定期更新并绘制双椭球体
- 标记交汇点位置

**使用示例**：
```matlab
[tree, path, success, metrics, fig_ellipsoid] = SC_RRT_with_Ellipsoid_Display(...
    startPoint, goalPoint, bounds, obstacles, ...
    'MaxIterations', 10000, 'Mode', 'pid', 'UpdateInterval', 50);
```

### 5. **test_ellipsoid_display.m** - 测试脚本
完整的测试脚本，演示椭球体显示功能。

**运行**：
```matlab
>> test_ellipsoid_display
```

**输出**：
- 环境配置窗口
- 椭球体动态显示窗口
- 性能指标输出
- 保存的图片文件

### 6. **INTEGRATION_GUIDE.m** - 集成指南
详细的集成方法文档（可读文件）。

## 使用方法

### 方法 A：运行测试脚本（推荐，快速体验）

```matlab
% 进入 Dynamic-RRT算法实现1-1 目录
cd Dynamic-RRT算法实现1-1

% 运行测试脚本
test_ellipsoid_display
```

### 方法 B：在 single_run_comparison.m 中集成

在 SC-RRT 算法运行部分，修改如下：

```matlab
elseif strcmp(alg.type, 'sc_rrt')
    % 启用椭球体显示
    if true  % 设为 false 禁用
        fig_temp = figure('Name', [alg.name ' - 椭球体显示'], ...
                         'Position', [950 100 900 800]);
        hold on; axis equal; grid on;
        xlim(config.bounds(1:2));
        ylim(config.bounds(3:4));
        
        % 绘制环境
        for i = 1:size(obstacles.circles, 1)
            circle = obstacles.circles(i, :);
            rectangle('Position', [circle(1)-circle(3), circle(2)-circle(3), ...
                                  2*circle(3), 2*circle(3)], ...
                     'Curvature', [1 1], 'FaceColor', [0.3 0.3 0.3]);
        end
        plot(config.startPoint(1), config.startPoint(2), 'go', 'MarkerSize', 12);
        plot(config.goalPoint(1), config.goalPoint(2), 'rs', 'MarkerSize', 12);
    else
        fig_temp = figure('Visible', 'off');
    end
    
    % 运行 SC_RRT_Bidirectional
    tic;
    [treeA, treeB, path, success, ~, metrics] = SC_RRT_Bidirectional(...
        config.startPoint, config.goalPoint, config.bounds, obstacles, ...
        fig_temp, '', 0, 0, ...
        'Mode', alg.params.Mode, ...
        'MaxIterations', config.max_iterations, ...
        'UseParetoFrontier', alg.params.UseParetoFrontier, ...
        'EnableVisualization', false);
    elapsed_time = toc;
```

### 方法 C：在 compare_algorithms.m 中集成

添加椭球体显示标志到算法配置：

```matlab
% SC-RRT 配置示例
algorithms{5} = struct(...
    'name', 'SC-RRT Adaptive', ...
    'type', 'sc_rrt', ...
    'params', struct('Mode', 'adaptive', 'UseParetoFrontier', true), ...
    'color', [0.00 0.45 0.74], ...
    'show_ellipsoid', true);  % 新增：启用椭球体显示

% 在运行循环中检查并处理
if alg.show_ellipsoid
    fprintf('  椭球体显示: 已启用\n');
    % 创建独立显示窗口
    fig_ellipsoid = figure(...);
end
```

## 椭球体显示说明

### 颜色含义
- **红色椭圆**：从起点到交汇点的采样约束（椭球体 A）
- **蓝色椭圆**：从交汇点到终点的采样约束（椭球体 B）
- **黄色菱形**：当前的动态交汇点

### 更新频率
- 默认每 50 次迭代更新一次椭球体显示
- 可通过 `UpdateInterval` 参数调整
- 更新间隔越小，显示越频繁（但性能开销更大）

### 数学原理

椭球体上的点 P 满足距离和等式：
$$||P - \text{focus1}|| + ||P - \text{focus2}|| = c_{\text{Best}}$$

其中：
- `focus1`, `focus2` 是椭球体的两个焦点
- `c_Best` 是焦点距离和（椭球约束参数）
- 椭球的半长轴：$a = \frac{c_{\text{Best}}}{2}$
- 椭球的半焦距：$c = \frac{||\text{focus2} - \text{focus1}||}{2}$
- 椭球的半短轴：$b = \sqrt{a^2 - c^2}$

## 性能考虑

### 椭球体显示对性能的影响
- 绘制 2D 椭圆：轻微影响（推荐）
- 绘制 3D 椭球：中等影响（50+ 个 mesh 点）
- 频繁更新：性能取决于系统配置

### 优化建议
1. 在 2D 空间中使用椭球体显示（性能更好）
2. 增加 `UpdateInterval` 减少更新频率
3. 关闭 MATLAB 的自动图形优化以获得更好的实时性
4. 在快速实验中禁用椭球体显示，仅在分析时启用

## 文件清单

| 文件名 | 类型 | 大小 | 说明 |
|--------|------|------|------|
| plotEllipsoid.m | 函数 | ~150 行 | 椭球体绘制 |
| calculateDualEllipsoidParams.m | 函数 | ~90 行 | 双椭球参数计算 |
| calculatePotentialMeetPoint.m | 函数 | ~60 行 | 交汇点计算 |
| SC_RRT_with_Ellipsoid_Display.m | 模块 | ~250 行 | 完整集成 |
| test_ellipsoid_display.m | 脚本 | ~200 行 | 测试脚本 |
| INTEGRATION_GUIDE.m | 文档 | ~150 行 | 集成指南 |
| README.md | 文档 | 本文件 | 说明文档 |

## 常见问题

### Q1: 椭球体不显示怎么办？
**A**: 检查以下项：
1. 确认 `plotEllipsoid.m` 在 MATLAB 路径中
2. 确认椭球体参数有限（`isfinite(c_best_A)` 等）
3. 检查图形窗口是否被其他窗口遮挡
4. 尝试手动调用 `drawnow` 强制刷新

### Q2: 椭球体显示很卡怎么办？
**A**: 
1. 增加 `UpdateInterval` 参数（例如改为 100）
2. 关闭其他窗口和应用
3. 在 3D 空间中使用更少的 mesh 点
4. 使用硬件加速渲染（改变 Renderer）

### Q3: 3D 椭球体不正确怎么办？
**A**: 
1. 检查旋转矩阵计算（Rodrigues 公式）
2. 验证椭球体参数不为负
3. 尝试减少 mesh 分辨率
4. 参考 `plotEllipsoid.m` 中的旋转矩阵验证注释

### Q4: 如何自定义椭球体颜色？
**A**: 修改调用时的颜色参数：
```matlab
% 自定义颜色（RGB 三元组）
plotEllipsoid(p1, p2, cBest, m, [R G B]);  % 例如 [1 0.5 0] 是橙色
```

## 贡献和改进

欢迎提出改进建议：
- 椭球体渲染效率
- 参数计算精度
- 用户界面增强
- 文档完善

## 许可证

遵循原 Dynamic-RRT 和 SC-RRT 项目的许可证。

## 更新日志

### v1.0 (2024-02)
- 初始版本
- 支持 2D/3D 椭球体显示
- 双椭球参数动态计算
- 完整的测试和集成脚本

---

**最后更新**: 2024年2月
**联系方式**: 请参考项目主目录
