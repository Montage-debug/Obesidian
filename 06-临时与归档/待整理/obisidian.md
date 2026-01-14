**工程级、可长期用、不靠运气的「Obsidian + Git + 多设备 零冲突规范」**。  
一套**约束 + 习惯 + 配置**的组合方案。

---

# 一、核心原则

> **Git 只管“内容”，不管“状态 / 布局 / 临时数据”**

|类型|是否进 Git|原因|
|---|---|---|
|Markdown 笔记|✅ 必须|真正的知识资产|
|图片 / 附件|✅ 必须|内容组成部分|
|Obsidian 工作区|❌ 严禁|每台设备都不同|
|UI 状态 / 缓存|❌ 严禁|高频变化、必冲突|
|插件配置|⚠️ 精选|只保留稳定、可复现的|

---

# 二、**标准目录结构

```
Obsidian_base/
├── .git/
├── .gitignore
├── .obsidian/
│   ├── core-plugins.json        ✅
│   ├── community-plugins.json  ✅
│   ├── app.json                ⚠️（可选）
│   ├── appearance.json         ⚠️（可选）
│   ├── snippets/               ❌
│   ├── plugins/                ❌
│   ├── workspace.json          ❌（你已经处理对了）
│   └── workspace-mobile.json   ❌
├── Assets/                      ✅ 图片、PDF
├── 机械革命/
├── ROS2/
└── README.md
```

---

# 三、**最终 `.gitignore

👉 **这是关键文件，建议你直接复制**

```gitignore
# ===============================
# Obsidian - 强制零冲突规则
# ===============================

# 工作区 / 布局（必忽略）
.obsidian/workspace.json
.obsidian/workspace-mobile.json

# 本地缓存 / 索引
.obsidian/cache/
.obsidian/index/
.obsidian/graph.json

# 插件运行时状态（不稳定）
.obsidian/plugins/
.obsidian/snippets/

# 本地 UI 状态
.obsidian/hotkeys.json
.obsidian/backlink.json
.obsidian/quick-switcher.json

# 系统杂项
.DS_Store
Thumbs.db
*.tmp
*.swp
```

📌 **原则**：

> **凡是“会自动变”的，一律不进 Git**

---

# 四、`.obsidian` 里「只允许进 Git 的文件」

你可以把这条当作**白名单**：

```text
.obsidian/
├── core-plugins.json
├── community-plugins.json
├── app.json              （可选）
├── appearance.json       （可选）
```

### 为什么这些可以进？

- 它们是**“功能选择”而不是“状态结果”**
    
- 多设备一致性是**好事**
    

---

# 五、**多设备协作铁律**

### ✅ 正确流程

#### 每次打开 Obsidian 前

```bash
git pull
```

#### 每次关闭 Obsidian 后

```bash
git status
git add .
git commit -m "update notes"
git push
```

---

### ❌ 严禁的行为

|行为|后果|
|---|---|
|两台设备同时编辑|必冲突|
|不 pull 直接写|必冲突|
|把 workspace.json 放进 Git|永久冲突|
|不看 `git status` 直接 pull|99% 出事|

---

# 六、**分支策略**

只用 **一个长期分支**：

```text
dev  ← 所有设备都只用这个
```

不要搞：

- feature 分支
    
- rebase
    
- 多人合并
    

> 你这是**个人知识库**，不是 Linux Kernel。

---

# 七、**一次性自检命令**

```bash
git status
git check-ignore -v .obsidian/workspace.json
```

你应该看到：

```
.obsidian/workspace.json  ignored
```

---

# 八、灾难恢复方案

### 如果哪天又炸了：

```bash
git rebase --abort  # 若在 rebase
git merge --abort   # 若在 merge
git reset --hard origin/dev
```

> 内容在远程，布局在本地  
> **永远不会双输**

