#Ubuntu

1. 修改root密码
```
sudo passwd root
```

2. 转为root用户。==注意！必须先转为root用户==
```
 sudo su
```

3. 以下两个文件中 **原先的计算机名改为新计算机名**
```
sudo gedit /etc/hostname 
sudo gedit /etc/hosts
```

4. 以下两个文件中**原先的用户名改为新用户名**
==但是/home/“原先用户名” 中的不能更改，若更改重启后，便登陆不了系统了。==
```
sudo gedit /etc/passwd
sudo gedit /etc/shadow
```

5. 以上步骤完毕后，重启，重启后，进入系统，==发现 home 目录下用户目录还是原先用户名，建议不要修改==，否则一些配置的.bashrc等文件设置的功能无法使用

6. 修改组文件，查找原先的用户名，全部修改为新用户名！ 
```
sudo gedit /etc/group
```



若不幸，没有提前进入root，而是一直使用sudo，导致后期无法修改shadow等其他文件，原因是前期修改了用户名，但是shadow文件里保存的用户名和密码还是原先的，但是用户名已修改为新的用户名，这就造成无论输入多么正确的密码都提示错误。

---
原文链接：https://blog.csdn.net/code_segment/article/details/78145594