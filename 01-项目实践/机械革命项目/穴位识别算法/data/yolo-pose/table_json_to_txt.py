import json
import os
from pathlib import Path
import glob

def convert_labelme_to_yolo_simple(json_file_path, output_dir=None):
    """
    将labelme标注转换为YOLO格式（简单版，适用于固定顺序标注）
    
    参数:
    - json_file_path: JSON文件路径
    - output_dir: 输出目录，如果为None则输出到JSON文件同目录
    """
    # 读取JSON文件
    with open(json_file_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    # 获取图像尺寸
    img_width = data["imageWidth"]
    img_height = data["imageHeight"]
    
    # 获取所有形状
    shapes = data["shapes"]
    
    # 按顺序处理：先找所有person，然后每个person后面按顺序找7个关键点
    yolo_lines = []
    keypoint_order = ["p"]
    
    # 遍历所有形状
    i = 0
    while i < len(shapes):
        shape = shapes[i]
        
        if shape["label"] == "navel":
            # 这是一个person对象，开始处理
            bbox_points = shape["points"]
            
            # 确保边界框有两个点
            if len(bbox_points) >= 2:
                # 获取边界框的坐标并确保正确顺序
                x1, y1 = bbox_points[0]
                x2, y2 = bbox_points[1]
                x1, x2 = min(x1, x2), max(x1, x2)
                y1, y2 = min(y1, y2), max(y1, y2)
                
                # 计算归一化的边界框参数
                x_center = (x1 + x2) / 2.0
                y_center = (y1 + y2) / 2.0
                width = x2 - x1
                height = y2 - y1
                
                x_center_norm = x_center / img_width
                y_center_norm = y_center / img_height
                width_norm = width / img_width
                height_norm = height / img_height
                
                # 开始构建YOLO格式行
                line_parts = ["0"]  # navel类别索引
                line_parts.append(f"{x_center_norm:.6f}")
                line_parts.append(f"{y_center_norm:.6f}")
                line_parts.append(f"{width_norm:.6f}")
                line_parts.append(f"{height_norm:.6f}")
                
                # 处理7个关键点
                # 初始化关键点字典，默认都缺失
                keypoints_found = {label: None for label in keypoint_order}
                
                # 检查person后面的形状是否有关键点
                j = i + 1
                while j < len(shapes) and shapes[j]["label"] != "navel":
                    # 如果这个形状是关键点
                    if shapes[j]["label"] in keypoint_order:
                        label = shapes[j]["label"]
                        points = shapes[j]["points"]
                        if points and len(points) > 0:
                            keypoints_found[label] = points[0]
                    j += 1
                
                # 按顺序添加7个关键点
                for label in keypoint_order:
                    if keypoints_found[label] is not None:
                        # 关键点存在
                        kp_x, kp_y = keypoints_found[label]
                        kp_x_norm = kp_x / img_width
                        kp_y_norm = kp_y / img_height
                        line_parts.append(f"{kp_x_norm:.6f}")
                        line_parts.append(f"{kp_y_norm:.6f}")
                        line_parts.append("2")  # 可见性为2
                    else:
                        # 关键点缺失
                        line_parts.append("0.0")
                        line_parts.append("0.0")
                        line_parts.append("0")  # 不可见
                
                # 将行添加到结果中
                yolo_lines.append(" ".join(line_parts))
            
            # 移动到下一个形状
            i += 1
        else:
            # 如果不是person，跳过（这种情况不应该发生，因为标注顺序是person在前）
            i += 1
    
    # 如果没有找到任何person对象
    if not yolo_lines:
        print(f"警告: {json_file_path} 中没有找到navel对象")
        return 0
    
    # 确定输出路径
    if output_dir:
        output_path = Path(output_dir) / Path(json_file_path).stem
    else:
        output_path = Path(json_file_path).with_suffix('.txt')
    
    # 写入YOLO格式文件
    with open(f"{output_path}.txt", 'w', encoding='utf-8') as f:
        f.write("\n".join(yolo_lines))
    
    return len(yolo_lines)


def convert_all_json_in_folder(input_dir, output_dir=None):
    """
    批量转换文件夹中的所有JSON文件
    """
    # 获取所有JSON文件
    json_files = glob.glob(os.path.join(input_dir, "*.json"))
    
    if not json_files:
        print(f"在目录 {input_dir} 中没有找到JSON文件")
        return
    
    print(f"找到 {len(json_files)} 个JSON文件")
    
    total_objects = 0
    successful_files = 0
    
    for json_file in json_files:
        try:
            filename = Path(json_file).name
            print(f"正在处理: {filename}", end="")
            
            objects_count = convert_labelme_to_yolo_simple(json_file, output_dir)
            
            if objects_count > 0:
                total_objects += objects_count
                successful_files += 1
                print(f" -> 成功，{objects_count}个对象")
            else:
                print(f" -> 失败，没有找到有效对象")
                
        except Exception as e:
            print(f" -> 错误: {str(e)}")
    
    print(f"\n转换完成！")
    print(f"成功处理: {successful_files}/{len(json_files)} 个文件")
    print(f"总共对象数: {total_objects}")
    print(f"平均每个文件对象数: {total_objects/successful_files if successful_files>0 else 0:.2f}")


if __name__ == "__main__":
    # 输入目录
    input_dir = "/home/cjk/myproject/Dataset/abdomen_dataset/labels/train_json"
    
    # 输出目录
    output_dir = "/home/cjk/myproject/Dataset/abdomen_dataset/labels/train"
    
    # 批量转换
    convert_all_json_in_folder(input_dir, output_dir)