# 配置
## 1、图形化界面添加连接
| 字段       | 填写内容                         |
| -------- | ---------------------------- |
| Name     | 随便填，比如 “My Linux PC”         |
| Host     | 输入目标主机的 IP，如 `192.168.1.100` |
| Port     | `4000`（默认）                   |
| Protocol | `NX`                         |
| Username | 留空或填目标主机的用户名（如 `wlzc`）✅ 可选   |
| Password | 留空 ✅ 可选                      |


=====================================================
```c
开启服务
1、sudo /etc/NX/nxserver --startup

配置防火墙
2、sudo ufw allow 4000/tcp

配置无头模式连接
3、sudo nano /usr/NX/etc/server.cfg
#Nomachine Configuration
CreateDisplay 1
DisplayOwner "wlzc"
NXVirtualDesktopSize 1920x1080

重启Nomchine server
4、sudo /etc/NX/nxserver --restart
```
=====================================================


## 2、终端界面添加连接

```c
1、连接账户
nx --connect 192.168.1.100 --user wlzc

2、输入密码
nx --connect 192.168.1.100 --user wlzc --password 你的密码
```

## 3、公网IP链接
1、获取公网IP
curl ifconfig.me

2、add host
输入IP内容---需打开防火墙
=====================================================
