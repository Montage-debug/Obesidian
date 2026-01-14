
```shell
sudo add-apt-repository ppa:linuxgndu/sqlitebrowser
sudo apt update
sudo apt install sqlitebrowser
```

```shell
# 查看所有表
  sqlite3 （这里采用绝对路径查询直至文件名）massage_head_config.db ".tables"

  # 查看表结构
  sqlite3 （这里采用绝对路径查询直至文件名）massage_head_config.db ".schema"

```
