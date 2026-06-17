import os

def rename_all_jpg_in_folder(folder_path):
    """将指定文件夹中的所有jpg图片重命名为img_1.jpg, img_2.jpg..."""
    
    # 获取所有jpg文件（包括大小写）
    all_files = os.listdir(folder_path)
    jpg_files = []
    
    for f in all_files:
        if f.lower().endswith(('.jpg', '.jpeg', '.jfif')):
            jpg_files.append(f)
    
    if not jpg_files:
        print("没有找到jpg图片")
        return
    
    # 按文件名排序
    jpg_files.sort()
    
    print(f"找到 {len(jpg_files)} 个jpg文件")
    print("开始重命名...")
    
    # 两步重命名法避免冲突
    # 第一步：重命名为临时文件
    temp_files = []
    for i, filename in enumerate(jpg_files, 1):
        old_path = os.path.join(folder_path, filename)
        temp_name = f"_temp_{i}.jpg"
        temp_path = os.path.join(folder_path, temp_name)
        
        os.rename(old_path, temp_path)
        temp_files.append((temp_path, i))
    
    # 第二步：重命名为最终文件名
    for temp_path, i in temp_files:
        final_name = f"img_{i}.jpg"
        final_path = os.path.join(folder_path, final_name)
        os.rename(temp_path, final_path)
        print(f"  {final_name}")
    
    print("重命名完成！")

# 直接设置你的文件夹路径
if __name__ == "__main__":
    # 在这里设置你的图片文件夹路径
    image_folder = "/home/cjk/myproject/Dataset/abdomen_dataset/images/train"  # 修改这里！
    
    # 检查路径是否存在
    if not os.path.exists(image_folder):
        print(f"错误：文件夹 '{image_folder}' 不存在")
    else:
        rename_all_jpg_in_folder(image_folder)