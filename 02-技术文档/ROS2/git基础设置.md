 

---

## **1️⃣ 修改 Git 配置（邮箱、用户名）**

在 VSCode 里打开终端（快捷键：`` Ctrl+` `` ），输入以下命令：

```bash
# 查看当前 Git 配置
`git config --global --list

# 修改全局用户名
git config --global user.name "你的名字"

# 修改全局邮箱
git config --global user.email "你的邮箱"
```

> 如果只想针对当前仓库修改，不加 `--global` 即可：
> 
> ```bash
> git config user.name "你的名字"
> git config user.email "你的邮箱"
> ```

检查修改是否生效：

```bash
git config user.name
git config user.email
```

---

## **2️⃣ 初始化或修改仓库位置**

### 如果你已经有仓库（远程仓库 URL 已知）：

```bash
# 进入项目文件夹
cd /你的项目路径

# 初始化本地仓库（如果还没初始化）
git init

# 添加远程仓库
git remote add origin https://github.com/用户名/仓库名.git

# 查看远程仓库是否添加成功
git remote -v
```

> 如果仓库已经存在远程地址，要修改 URL：
> 
> ```bash
> git remote set-url origin https://github.com/用户名/仓库名.git
> ```

---

## **3️⃣ 上传文件到远程仓库**

1. **查看文件状态**
    

```bash
git status
```

2. **添加文件到暂存区**
    

```bash
git add .
```

> `.` 表示添加当前文件夹下所有修改过的文件，你也可以指定单个文件：
> 
> ```bash
> git add 文件名
> ```

3. **提交文件**
    

```bash
git commit -m "提交说明，比如：第一次提交"
```

4. **推送到远程仓库**
    

```bash
git push -u origin main
```

> 注意：如果你的默认分支是 `master`，就用 `master` 替换 `main`。

---

### ✅ **完整流程示例**

```bash
# 进入项目
cd /path/to/project

# 修改用户名和邮箱
git config user.name "Guo"
git config user.email "guo@example.com"

# 初始化仓库
git init

# 添加远程仓库
git remote add origin https://github.com/guo/myrepo.git

# 添加文件
git add .

# 提交
git commit -m "Initial commit"

# 推送到远程
git push -u origin main
```

---

如果你需要，我可以帮你画一张 **VSCode 操作 Git 的流程图**，让整个上传流程一目了然，包括修改邮箱、添加远程仓库、上传文件。

你希望我画吗？



```c
# windows合并
git merge origin/dev
```

```

|命令|主要用途|输出示例|
|---|---|---|
|`git remote -v`|**查看所有远程仓库**的名称和URL[](https://worktile.com/kb/ask/252138.html)[](https://worktile.com/kb/ask/237040.html)|`origin https://github.com/user/repo.git (fetch)`  <br>`origin https://github.com/user/repo.git (push)`|
|`git remote get-url <远程名称>`|获取**单个指定远程仓库**的URL[](https://cloud.tencent.cn/developer/information/%e5%a6%82%e4%bd%95%e6%9f%a5%e7%9c%8b%e5%b7%a5%e4%bd%9c%e7%9b%ae%e5%bd%95%e6%89%80%e6%8c%87%e5%90%91%e7%9a%84%e5%ad%98%e5%82%a8%e5%ba%93URL%ef%bc%9f-salon)|`https://github.com/user/repo.git`|
|`git config --get remote.<远程名称>.url`|直接查询Git配置中**特定远程的URL**[](https://worktile.com/kb/ask/252138.html)[](https://worktile.com/kb/ask/237040.html)|`https://github.com/user/repo.git`|
|`git remote show <远程名称>`|显示**指定远程仓库的详细信息**，包括其URL[](https://worktile.com/kb/ask/237040.html)|_输出包含URL、跟踪分支等详细信息_|
```