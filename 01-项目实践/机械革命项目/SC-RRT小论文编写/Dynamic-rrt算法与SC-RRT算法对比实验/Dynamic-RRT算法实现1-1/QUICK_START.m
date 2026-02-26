% =========================================================================
%                     快速开始指南
%          SC-RRT 椭球体动态显示功能 - Quick Start
% =========================================================================

% 本文件提供了最快的方式来开始使用椭球体显示功能

fprintf('\n');
fprintf('╔════════════════════════════════════════════════════════════╗\n');
fprintf('║                SC-RRT 椭球体动态显示功能                   ║\n');
fprintf('║                  快速开始指南 v1.0                        ║\n');
fprintf('╚════════════════════════════════════════════════════════════╝\n\n');

fprintf('📋 已为您安装的新功能模块:\n\n');

fprintf('1️⃣  plotEllipsoid.m\n');
fprintf('    • 椭球体/椭圆绘制函数（支持2D和3D）\n');
fprintf('    • 用法: h = plotEllipsoid(focus1, focus2, cBest, m, color)\n\n');

fprintf('2️⃣  calculateDualEllipsoidParams.m\n');
fprintf('    • 计算双椭球体约束参数\n');
fprintf('    • 用法: [cA, cB, cmA, cmB] = calculateDualEllipsoidParams(...)\n\n');

fprintf('3️⃣  calculatePotentialMeetPoint.m\n');
fprintf('    • 动态计算两树的交汇点\n');
fprintf('    • 用法: [meet, cA, cB] = calculatePotentialMeetPoint(...)\n\n');

fprintf('4️⃣  SC_RRT_with_Ellipsoid_Display.m\n');
fprintf('    • SC-RRT完整集成模块\n');
fprintf('    • 用法: [tree, path, success, metrics, fig] = SC_RRT_with_Ellipsoid_Display(...)\n\n');

fprintf('5️⃣  test_ellipsoid_display.m\n');
fprintf('    • 完整的测试脚本\n\n');

fprintf('═════════════════════════════════════════════════════════════\n\n');

fprintf('🚀 最快的使用方法（3步）:\n\n');

fprintf('   步骤1: 打开MATLAB命令行\n');
fprintf('   >> cd Dynamic-RRT算法实现1-1\n\n');

fprintf('   步骤2: 运行测试脚本\n');
fprintf('   >> test_ellipsoid_display\n\n');

fprintf('   步骤3: 观察输出\n');
fprintf('   • 环境配置窗口\n');
fprintf('   • 椭球体动态显示窗口\n');
fprintf('   • 控制台的性能指标\n\n');

fprintf('═════════════════════════════════════════════════════════════\n\n');

fprintf('📊 椭球体显示说明:\n\n');

fprintf('颜色含义:\n');
fprintf('  🔴 红色椭圆  = 起点 → 交汇点 的采样约束\n');
fprintf('  🔵 蓝色椭圆  = 交汇点 → 终点 的采样约束\n');
fprintf('  🟡 黄色菱形  = 当前的动态交汇点\n');
fprintf('  ⚫ 黑色圆    = 障碍物\n\n');

fprintf('═════════════════════════════════════════════════════════════\n\n');

fprintf('🔧 集成到现有对比实验:\n\n');

fprintf('方法A - 在 single_run_comparison.m 中:\n\n');

fprintf('    在第 165-210 行找到 SC-RRT 的运行部分\n');
fprintf('    修改以下代码:\n\n');

fprintf('    === 修改前 ===\n');
fprintf('    elseif strcmp(alg.type, ''sc_rrt'')\n');
fprintf('        fig_temp = figure(''Visible'', ''off'');\n\n');

fprintf('    === 修改后 ===\n');
fprintf('    elseif strcmp(alg.type, ''sc_rrt'')\n');
fprintf('        fig_temp = figure(''Name'', [alg.name '' - 椭球体显示''],\n');
fprintf('                         ''Position'', [950 100 900 800]);\n');
fprintf('        hold on; axis equal; grid on;\n');
fprintf('        xlim(config.bounds(1:2));\n');
fprintf('        ylim(config.bounds(3:4));\n\n');

fprintf('方法B - 创建专用脚本:\n\n');

fprintf('    创建 demo_with_ellipsoid.m:\n\n');

fprintf('    clear; clc; close all;\n');
fprintf('    config.bounds = [0 1500 0 1500];\n');
fprintf('    config.startPoint = [400 400];\n');
fprintf('    config.goalPoint = [1100 1100];\n');
fprintf('    config.numObstacles = 225;\n');
fprintf('    config.seed = 42;\n\n');

fprintf('    rng(config.seed);\n');
fprintf('    obstacles = generateObstacles(''2D'', config.bounds, ...\n');
fprintf('                                 config.numObstacles, 15, ...\n');
fprintf('                                 config.startPoint, config.goalPoint);\n\n');

fprintf('    [tree, path, success, metrics, fig_ellipsoid] = ...\n');
fprintf('        SC_RRT_with_Ellipsoid_Display(...\n');
fprintf('        config.startPoint, config.goalPoint, config.bounds, obstacles, ...\n');
fprintf('        ''MaxIterations'', 10000, ''Mode'', ''pid'');\n\n');

fprintf('═════════════════════════════════════════════════════════════\n\n');

fprintf('📚 详细文档:\n\n');

fprintf('  • ELLIPSOID_DISPLAY_README.md - 完整的说明文档\n');
fprintf('  • INTEGRATION_GUIDE.m - 集成方法详解\n\n');

fprintf('═════════════════════════════════════════════════════════════\n\n');

fprintf('⚡ 关键参数配置:\n\n');

fprintf('函数: SC_RRT_with_Ellipsoid_Display(...)\n\n');

fprintf('  必需参数:\n');
fprintf('    startPoint    - 起始点 [1×m]\n');
fprintf('    goalPoint     - 目标点 [1×m]\n');
fprintf('    bounds        - 规划空间边界 [1×2m]\n');
fprintf('    obstacles     - 障碍物结构体\n\n');

fprintf('  可选参数:\n');
fprintf('    ''MaxIterations'', 10000   - 最大迭代次数\n');
fprintf('    ''Mode'', ''pid''             - 算法模式 (basic|pid|adaptive)\n');
fprintf('    ''UpdateInterval'', 50      - 椭球体更新间隔\n\n');

fprintf('示例:\n');
fprintf('    [tree, path, success, metrics, fig] = SC_RRT_with_Ellipsoid_Display(\n');
fprintf('        [400 400], [1100 1100], [0 1500 0 1500], obstacles, ...\n');
fprintf('        ''MaxIterations'', 10000, ...\n');
fprintf('        ''Mode'', ''adaptive'', ...\n');
fprintf('        ''UpdateInterval'', 50);\n\n');

fprintf('═════════════════════════════════════════════════════════════\n\n');

fprintf('❓ 常见问题:\n\n');

fprintf('Q1: 椭球体不显示?\n');
fprintf('    → 检查 plotEllipsoid.m 是否在路径中\n');
fprintf('    → 确认图形窗口没有被遮挡\n');
fprintf('    → 尝试手动运行: drawnow\n\n');

fprintf('Q2: 显示很卡?\n');
fprintf('    → 增加 UpdateInterval 参数\n');
fprintf('    → 关闭其他窗口\n');
fprintf('    → 禁用路径平滑功能\n\n');

fprintf('Q3: 如何禁用椭球体显示?\n');
fprintf('    → 在 single_run_comparison.m 中设置: enable_ellipsoid = false;\n');
fprintf('    → 或创建不使用显示窗口的版本\n\n');

fprintf('═════════════════════════════════════════════════════════════\n\n');

fprintf('📝 文件结构:\n\n');

fprintf('Dynamic-RRT算法实现1-1/\n');
fprintf('├── plotEllipsoid.m                       ✨ 椭球体绘制\n');
fprintf('├── calculateDualEllipsoidParams.m        ✨ 双椭球参数\n');
fprintf('├── calculatePotentialMeetPoint.m         ✨ 交汇点计算\n');
fprintf('├── SC_RRT_with_Ellipsoid_Display.m       ✨ 完整集成\n');
fprintf('├── test_ellipsoid_display.m              ✨ 测试脚本\n');
fprintf('├── ELLIPSOID_DISPLAY_README.md           📖 完整文档\n');
fprintf('├── INTEGRATION_GUIDE.m                   📖 集成指南\n');
fprintf('├── QUICK_START.m                         📖 本文件\n');
fprintf('└── ...\n\n');

fprintf('═════════════════════════════════════════════════════════════\n\n');

fprintf('🎯 后续步骤:\n\n');

fprintf('1. 运行 test_ellipsoid_display 体验椭球体显示\n');
fprintf('2. 查看 ELLIPSOID_DISPLAY_README.md 了解详细信息\n');
fprintf('3. 参考 INTEGRATION_GUIDE.m 集成到对比实验\n');
fprintf('4. 修改参数优化显示效果\n\n');

fprintf('═════════════════════════════════════════════════════════════\n\n');

fprintf('💡 提示:\n\n');

fprintf('• 椭球体显示对于理解 SC-RRT 的采样策略非常有帮助\n');
fprintf('• 在论文中可以截图展示椭球体的动态演变过程\n');
fprintf('• 可以对比不同参数下椭球体的形状变化\n');
fprintf('• 支持 2D 和 3D 两种显示模式\n\n');

fprintf('═════════════════════════════════════════════════════════════\n\n');

fprintf('现在就可以开始了! 🚀\n\n');

fprintf('>> test_ellipsoid_display\n\n');

fprintf('═════════════════════════════════════════════════════════════\n\n');
