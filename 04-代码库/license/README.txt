WLZC 按摩机器人 - 授权工具
========================================

使用方法:

1. 快速激活（推荐）:
   ./activate.sh

2. 查看帮助:
   ./wlzc_license_tool/run.sh --help

3. 获取机器码:
   ./wlzc_license_tool/run.sh get-machine-code

4. 生成授权文件:
   ./wlzc_license_tool/run.sh generate --customer "客户名称" --company "公司名称"

5. 检查已安装授权:
   ./wlzc_license_tool/run.sh check-installed

注意事项:
- 所有命令均无需 root 权限
- 授权文件将自动安装到 /etc/wlzc/license/
- 如遇权限问题，请联系系统管理员

========================================
WLZC Technology
