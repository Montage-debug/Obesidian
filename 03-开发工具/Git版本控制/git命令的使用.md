#git

https://blog.csdn.net/u011709538/article/details/132618844
> 常用GIT命令详解，手把手让你登堂入室


## 核心命令

### ==config 查看和配置==
```
# 查看配置列表
git config --list
```

```
# 分别在全局和本地设置用户名
git config --global user.name qihemu
git config --local user.name qihemu

git config --global user.email you@example.com

# 查看 user.name
git config user.name
```


### ==git init 创建仓库==
```
git init
```
需要注意的是，`git init` 命令只在仓库初始化时使用一次，创建仓库后就不需要重复执行。

### ==git clone 下载仓库==

```
git clone https://项目地址.git
```

### ==git add 添加至暂存区==
```
# 逐个添加文件
git add ./file1.txt ./temp/file2.txt

# 不带参数，只有目录，代表把本目录所有修改过，和新建的文件加入暂存区。
# 但被你删除的文件不会
# 通常用这句就行！！
git add .

# 更新暂存区已有的，即包括修改过的，和被你删除的，但不包括新建的文件
git add -u
# All 代表所有文件，包括你新增的、修改的、删除的
git add -A
```


### ==git commit 命令==

将暂存区的内容提交至本地仓库
> 首先得使用git add命令

```
# 如果暂存区已经有了内容，接下来就可以将暂存区的内容提交至本地仓库
git commit

# 提交时添加提交信息
git commit -m "这是我的第一次提交"
```


### ==git push 推送==

```
git push <远程主机名> <本地分支名>:<远程分支名>

# 常用
git push origin dev

```

一些常用参数
```
# 设置远程仓库的 dev 分支为默认 push 目标，后续 push 可以省略参数
git push -u origin dev

# 强制推送，即将本地仓库的修改强制覆盖远程仓库的修改
git push -f origin dev

# 推送所有分支到远程仓库，包括新分支和删除的分支
git push --all origin

# 删除远程仓库的 dev 分支
git push origin --delete dev
```


### ==git pull 拉取==
```
git pull <远程主机名> <远程分支名>:<本地分支名>
```
常用
```
git pull origin dev
```
需要注意的是，pull会同时更新本地工作区，但不会更新暂存区。
同时，因为这里有着一个合并操作,
所以其有着两种方式合并：`rebase` 或者 `merge`(默认)

```
# 将远程仓库的提交合并到本地仓库时使用 rebase （变基）方式，以使合并的提交历史更清晰和整洁
git pull --rebase origin dev

```


### ==git branch分支管理命令==

主要用法：
```
git branch：列出所有本地分支。
git branch -r：列出所有远程分支。
git branch -a：列出所有本地和远程分支。
git branch <branchname>：创建一个新的分支。
git branch -d <branchname>：删除一个分支。
git branch -D <branchname>：强制删除一个分支。
git branch -m <newbranchname>：重命名一个分支。
git branch -vv：显示本地分支与远程分支的对应关系。
git branch -u <upstream>：将本地分支与远程分支建立跟踪关系。
git branch --merged：列出已经合并到当前分支的分支。
git branch --no-merged：列出未合并到当前分支的分支。
git branch --contains <commit>：列出包含指定提交的分支。
git branch --set-upstream-to=<upstream>：设置当前分支跟踪另一个远程分支。
```

###  ==git checkout 切换分支==

```
git checkout <branch_name>

```

创建新分支并切换到该分支，相当于执行以下两个命令：`git branch <new_branch_name>` 和 `git checkout <new_branch_name>`

```
git checkout -b <branch_name>
```

### ==git rm删除文件/文件夹==

删除单个文件
```
# 本地中该文件不会被删除
git rm --cached <文件> 		
git commit -m '删除某个文件'
git push （origin master）
```

删除文件夹
```
# 删除文件夹
git rm -r --cached  <文件夹> 
git commit -m '删除某个文件'
git push （origin master）
```

###  ==git rebase==

先解决冲突再保存
```
git add .
git rebase --continue
```

如果 rebase 过程中，你想中途退出，恢复 rebase 前的代码则可以用命令
```
git rebase --abort
```

###  ==git fetch 重新拉取远程代码==

先获取远程最新信息
```
git fetch origin
```

然后重置到远程分支状态
```
git reset --hard origin/分支名
```

## 类似命令区别对比

### pull ＆ fetch

- ==fetch== 只是**将远程代码库的代码更新到本地代码库，不会自动合并代码**

- ==pull== 会首先执行`fetch`命令，然后自动将远程代码库的代码合并到本地代码库。如果有冲突，需要手动解决冲突并提交更改。如果没有冲突，pull会自动提交合并的更改。

## 常规更新远程仓库流程

确认当前分支
```
git branch dev
```

 拉取远程仓库内容
```
git pull origin dev
```

添加更新文件至暂存区
```
git add . 
```

提交暂存区内容以及备注信息
```
git commit -m "备注内容"
```

推送
```
git push origin dev
```

