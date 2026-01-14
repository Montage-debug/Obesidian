# 03-开发工具

本目录包含各类开发工具的配置指南和使用文档。

---

## 🛠️ 工具列表

### 1. Git版本控制
代码版本管理和协作工具

**包含内容：**
- Git基础命令
- Git配置设置
- 分支管理策略
- 团队协作规范

**常用命令：**
```bash
git clone <repo>
git add .
git commit -m "message"
git push origin <branch>
```

📁 [进入Git目录](./Git版本控制/)

---

### 2. MobaXterm
SSH远程连接和终端工具

**包含内容：**
- MobaXterm配置
- SSH连接设置
- VNC远程桌面
- 文件传输

**适用场景：** Linux远程开发、机器人系统调试

📁 [进入MobaXterm目录](./MobaXterm/)

---

### 3. PlatformIO
嵌入式开发IDE

**包含内容：**
- PlatformIO安装配置
- 工程创建流程
- 库管理
- 调试技巧

**支持平台：** STM32, ESP32, Arduino等

📁 [进入PlatformIO目录](./PlatformIO/)

---

### 4. Docker
容器化技术与部署

**包含内容：**
- Docker安装
- 镜像管理
- 容器部署
- Docker Compose

**适用场景：** 环境隔离、微服务部署

📁 [进入Docker目录](./Docker/)

---

## 🔧 工具配置建议

### 开发环境配置顺序
1. **基础工具**
   - Git版本控制
   - 代码编辑器 (VS Code)
   
2. **远程工具**
   - MobaXterm (Windows)
   - SSH配置
   
3. **专用IDE**
   - PlatformIO (嵌入式)
   - ROS2工具链
   
4. **容器技术**
   - Docker
   - Docker Compose

---

## 📚 相关资源

### Git学习资源
- [Git官方文档](https://git-scm.com/doc)
- [Pro Git电子书](https://git-scm.com/book/zh/v2)

### Docker学习资源
- [Docker官方文档](https://docs.docker.com/)
- [Docker Hub](https://hub.docker.com/)

---

## 💡 使用技巧

### Git最佳实践
- 提交前先pull最新代码
- 提交信息要清晰明确
- 使用分支进行功能开发
- 定期同步远程仓库

### MobaXterm技巧
- 保存SSH会话配置
- 使用SSH密钥免密登录
- 配置端口转发
- 使用多标签管理多个连接

### Docker技巧
- 使用.dockerignore减小镜像大小
- 多阶段构建优化镜像
- 使用Docker Compose管理多容器
- 定期清理未使用的镜像和容器

---

## 🔄 工具更新

| 工具 | 推荐版本 | 更新频率 |
|------|---------|---------|
| Git | 2.40+ | 稳定版 |
| MobaXterm | 最新版 | 根据需要 |
| PlatformIO | 6.0+ | 定期更新 |
| Docker | 24.0+ | 定期更新 |

---

[返回主目录](../README.md)
