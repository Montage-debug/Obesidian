
```c
文档文件入口
http://www.orangepi.cn/html/hardWare/computerAndMicrocontrollers/service-and-support/Orange-Pi-5-plus.html
```

---

# 1.查SSD卡是否挂载
lsblk
```
“orangepi@orangepi5plus:~$ lsblk
NAME        MAJ:MIN RM   SIZE RO TYPE MOUNTPOINTS
mtdblock0    31:0    0    16M  0 disk 
mmcblk1     179:0    0  58.2G  0 disk 
├─mmcblk1p1 179:1    0     1G  0 part /boot
└─mmcblk1p2 179:2    0  56.6G  0 part /var/log.hdd
                                      /
zram0       254:0    0   3.9G  0 disk [SWAP]
zram1       254:1    0   200M  0 disk /var/log
nvme0n1     259:0    0 119.2G  0 disk   // SSD磁盘所在
orangepi@orangepi5plus:~$ 
```
# 2.执行烧录指令 TF -> SSD
sudo dd if=/home/orangepi/Documents/Orangepi5plus_1.2.0_ubuntu_jammy_desktop_xfce_linux6.1.43.img of=/dev/nvme0n1 bs=4M status=progress conv=fsync
```
if=...：输入文件，指向你拥有的原始 .img 镜像。
of=/dev/nvme0n1：输出文件，指向你的 NVMe SSD 整盘（注意不要写成 nvme0n1p1 等分区）。
bs=4M：块大小，提高拷贝速度。
status=progress：实时显示进度条。
conv=fsync：确保数据在命令结束前完全物理写入磁盘，防止断电损坏。
```
# 3.同步数据
sudo sync

# 4.查看分区情况
查看 SSD 分区情况（可选，确认是否成功写入分区）：
```
lsblk
此时你应该能看到 nvme0n1 下面出现了类似 nvme0n1p1 和 nvme0n1p2 的分区，大小与 TF 卡类似。
```

# 5.重启
```
sudo poweroff
```

# 6.拔出TF卡后开机，采用官方自动扩容脚本
```
sudo orangepi-expand-fs
# 或者
sudo resize-rootfs
```

# 7.验证结果
```
再次输入 df -h，你应该能看到 / 挂载点的空间变成了 110G+ (即 128GB SSD 的实际可用空间)
```