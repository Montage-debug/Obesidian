```
#!/bin/bash

# 退出脚本，如果出现错误
set -e

# 更新安装包列表并安装 Samba
echo "更新安装包列表并安装 Samba ..."
sudo apt update
sudo apt install -y samba

# 创建共享目录并设置权限
echo "创建共享目录并设置权限 ..."
sudo mkdir -p /srv/samba/share
sudo chown -R nobody:nogroup /srv/samba/share
sudo chmod 2775 /srv/samba/share
sudo chown -R $USER:$USER /srv/samba/share

# 设置 Samba 密码
echo "设置 Samba 密码 ..."
sudo smbpasswd -a $USER
sudo smbpasswd -e $USER

# 配置 Samba 配置文件
echo "配置 Samba 配置文件 ..."
{
  echo "[share]"
  echo "   path = /srv/samba/share"
  echo "   browseable = yes"
  echo "   read only = no"
  echo "   guest ok = yes"
  echo "   force user = nobody"
  echo "   create mask = 0664"
  echo "   directory mask = 2775"
} | sudo tee -a /etc/samba/smb.conf > /dev/null

# 启动并使 Samba 服务开机启动
echo "启动并启用 Samba 服务 ..."
sudo systemctl enable --now smbd nmbd

# 重启 Samba 服务
echo "重启 Samba 服务 ..."
sudo systemctl restart smbd nmbd

# 配置防火墙允许 Samba 端口
echo "配置防火墙 ..."
sudo ufw allow Samba

# 安装 smbclient 工具用于测试
echo "安装 smbclient 工具 ..."
sudo apt install -y smbclient

# 验证 Samba 配置
echo "验证 Samba 配置 ..."
testparm -s
sudo smbclient -L localhost -U "$USER"

echo "Samba 服务端设置完成！"

```

```
允许所有用户可读写
sudo chmod 777 /srv/samba/share
chmod +x setup_samba.sh

```