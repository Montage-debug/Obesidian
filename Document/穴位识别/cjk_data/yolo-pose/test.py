import torch
import numpy as np
from ultralytics import YOLO
import os
from pathlib import Path 
import cv2
import time
from draw_point import get_model_path

start_time = time.time()
INPUT_DIR =  "./leg_data"  #"./test_data"    # 存放16张测试图片的文件夹
OUTPUT_DIR = "./test_result"  #"./test_result"    # 保存结果的文件夹

# image_extensions = (".jpg", ".jpeg", ".png", ".bmp")
# image_files = [f for f in os.listdir(INPUT_DIR) if f.lower().endswith(image_extensions)]

# model = YOLO(get_model_path("yolo11x-pose.pt"))

# model_name = "x"


# results = model("leg_data/leg_2.jpg")


# result = results[0]

# kpts = result.keypoints.data.cpu().numpy()
# # bbox = result.boxes.data.cpu().numpy()
# # print(bbox)

# for person_id, kpts_data in enumerate(kpts):
#     for kpt_id, (x, y, conf) in enumerate(kpts_data):
#         if conf > 0.5:  # 只显示置信度大于0.5的关键点
#             print(f"  关键点 {kpt_id}: 坐标({x:.1f}, {y:.1f}), 置信度{conf:.2f}")

# result.save(filename="test_result/x/leg_2.jpg")
# end_time = time.time()
# elapsed_time = end_time - start_time
# print(f"运行时间：{elapsed_time:.2f}秒")

# model_output_dir = os.path.join(OUTPUT_DIR, model_name)
# Path(model_output_dir).mkdir(exist_ok=True) 

# for img_idx, img_file in enumerate(image_files):
#     img_path = os.path.join(INPUT_DIR, img_file)
#     img_name = os.path.splitext(img_file)[0]  # 图片名称（无后缀）
#     print(f"\n--- 处理图片: {img_file} (模型: {model_name}) ---")
#     # 执行姿态识别
#     results = model(img_path)
#     yolo_results = model_yolo(img_path)

#     output_img_name = f"{img_name}_result.jpg"
#     output_img_path = os.path.join(model_output_dir, output_img_name)
    

#     result = results[0]
#     yolo_result = yolo_results[0]
#     yolo_result.save(filename=output_img_path)

#     yolo_kpts_data = yolo_result.keypoints.data[0].cpu().numpy()
#     kpts_data = result.keypoints.data[0].cpu().numpy()

#     img_with_points = cv2.imread(output_img_path)

#     color = (0,0,255)   
#     for kpt_id, (x, y, conf) in enumerate(kpts_data):

#         x_int, y_int = int(round(x)), int(round(y))
    
#         cv2.circle(img_with_points, (x_int, y_int), radius=8, 
#                     color=color, thickness=-1) 
#     left_hip = kpts_data[4]
#     right_hip = kpts_data[5]
#     mid_hip_x = (left_hip[0] + right_hip[0]) / 2
#     mid_hip_y = (left_hip[1] + right_hip[1]) / 2
#     distance_pixels = np.sqrt((left_hip[0] - right_hip[0])**2 + 
#                     (left_hip[1] - right_hip[1])**2)
#     cun = distance_pixels / 8.0
#     pred_navel_y = mid_hip_y - cun*3.5
#     pred_navel_x = mid_hip_x

#     cv2.circle(img_with_points, (int(round(mid_hip_x)), int(round(mid_hip_y))), radius=8, 
#                 color=color, thickness=-1)
#     cv2.circle(img_with_points, (int(round(pred_navel_x)), int(round(pred_navel_y))), radius=8, 
#                 color=(0,255,0), thickness=-1)
#     cv2.circle(img_with_points, (int(round(pred_navel_x)), int(round(mid_hip_y-cun))), radius=8, 
#                 color=(0,255,255), thickness=-1)
    
#     yolo_left_hip = yolo_kpts_data[11]
#     yolo_right_hip = yolo_kpts_data[12]
#     yolo_mid_hip_x = (yolo_left_hip[0] + yolo_right_hip[0]) / 2
#     yolo_mid_hip_y = (yolo_left_hip[1] + yolo_right_hip[1]) / 2
#     cv2.circle(img_with_points, (int(round(yolo_mid_hip_x)), int(round(yolo_mid_hip_y))), radius=8, 
#                 color=(255,0,0), thickness=-1)

#     cv2.imwrite(output_img_path, img_with_points)

# for img_idx, img_file in enumerate(image_files):
#     img_path = os.path.join(INPUT_DIR, img_file)
#     img_name = os.path.splitext(img_file)[0]  # 图片名称（无后缀）
#     print(f"\n--- 处理图片: {img_file} (模型: {model_name}) ---")
#     # 执行姿态识别
#     results = model(img_path)

#     output_img_name = f"{img_name}_result.jpg"
#     output_img_path = os.path.join(model_output_dir, output_img_name)

#     result = results[0]
#     kpts_data = result.keypoints.data[0].cpu().numpy()

#     left_hip = kpts_data[4]
#     right_hip = kpts_data[5]
#     mid_hip_x = (left_hip[0] + right_hip[0]) / 2
#     mid_hip_y = (left_hip[1] + right_hip[1]) / 2
#     distance_pixels = np.sqrt((left_hip[0] - right_hip[0])**2 + 
#                         (left_hip[1] - right_hip[1])**2)

#     orig_img = result.orig_img
#     img_with_points = orig_img.copy()
#     color = (0,0,255)   
#     for kpt_id, (x, y, conf) in enumerate(kpts_data):

#         x_int, y_int = int(round(x)), int(round(y))
    
#         cv2.circle(img_with_points, (x_int, y_int), radius=8, 
#                     color=color, thickness=-1) 
#     cv2.circle(img_with_points, (mid_hip_x, mid_hip_y), radius=8, 
#                     color=color, thickness=-1)

#     result.save(filename=output_img_path)

    
#     #画左右手腕的座标
#     kpts_data = result.keypoints.data[0].cpu().numpy()
#     left_shoulder = kpts_data[5]
#     right_shoulder = kpts_data[6]
#     left_hip = kpts_data[11]
#     right_hip = kpts_data[12]
#     # 4. 计算中点坐标
#     # 肩中点 (颈部点)
#     shoulder_mid_x = (left_shoulder[0] + right_shoulder[0]) / 2
#     shoulder_mid_y = (left_shoulder[1] + right_shoulder[1]) / 2
#     shoulder_mid = np.array([shoulder_mid_x, shoulder_mid_y, 
#                              min(left_shoulder[2], right_shoulder[2])])  # 取较低的置信度
    
#     # 髋中点 (腰部中心点)
#     hip_mid_x = (left_hip[0] + right_hip[0]) / 2
#     hip_mid_y = (left_hip[1] + right_hip[1]) / 2
#     hip_mid = np.array([hip_mid_x, hip_mid_y, 
#                         min(left_hip[2], right_hip[2])])  # 取较低的置信度
#     #寸的基准,分为两种，分为背部的骨度法和肩部分寸法。
#     #第一种背部骨度法
#     CUN_SHOULDER_TO_MIDLINE = 18
#     distance_pixels = np.sqrt((hip_mid_x - shoulder_mid_x)**2 + 
#                             (hip_mid_y - shoulder_mid_y)**2)
#     PIXELS_PER_CUN = distance_pixels / CUN_SHOULDER_TO_MIDLINE
#     #第二种肩部分寸法
#     CUN_SHOULDER_TO_MIDLINE_2 = 8.5
#     distance_pixels_1 = np.sqrt((right_shoulder[0] - shoulder_mid_x)**2 + 
#                           (right_shoulder[1] - shoulder_mid_y)**2)
#     PIXELS_PER_CUN_2 = distance_pixels / CUN_SHOULDER_TO_MIDLINE_2

#     cun_bi = PIXELS_PER_CUN / PIXELS_PER_CUN_2
#     #shoulder_mid到hip_mid的距离除去left_shoulder到right_shoulder的距离得到比值
#     RLS_distance = np.sqrt((right_shoulder[0]-left_shoulder[0])**2+(right_shoulder[1]-left_shoulder[1])**2)
#     SHM_bi_RLS = distance_pixels /RLS_distance
#     RLH_distance = np.sqrt((right_hip[0]-left_hip[0])**2+(right_hip[1]-left_hip[1])**2)
#     SHM_bi_RLH = distance_pixels/RLH_distance
#     print(f"这是肩宽于背长比:{SHM_bi_RLS} 胯宽于背长比:{SHM_bi_RLH}")

#     # 1. 计算肩部连线角度
#     shoulder_vector = np.array([right_shoulder[0] - left_shoulder[0],
#                                right_shoulder[1] - left_shoulder[1]])
#     shoulder_angle = np.degrees(np.arctan2(shoulder_vector[1], shoulder_vector[0]))
    
#     # 2. 计算髋部连线角度
#     hip_vector = np.array([right_hip[0] - left_hip[0],
#                           right_hip[1] - left_hip[1]])
#     hip_angle = np.degrees(np.arctan2(hip_vector[1], hip_vector[0]))

#     # 3. 计算身体中线角度（肩中点到髋中点）
#     body_vector = np.array([hip_mid[0] - shoulder_mid[0],
#                            hip_mid[1] - shoulder_mid[1]])
#     body_angle = np.degrees(np.arctan2(body_vector[1], body_vector[0]))
    
#     print(f"肩膀连线角度:{abs(shoulder_angle):.1f}度")
#     print(f"髋部连线角度:{abs(hip_angle):.1f}度")
#     print(f"身体中线角度:{abs(body_angle):.1f}度")

    # left_wrist = kpts[9]
    # right_wrist = kpts[10]
    # x_l, y_l, conf_l = left_wrist
    # x_r, y_r, conf_r = right_wrist
    # int_x_l, int_y_l = int(torch.round(x_l)),int(torch.round(y_l))
    # int_x_r, int_y_r = int(torch.round(x_r)),int(torch.round(y_r))
    # img = cv2.imread(output_img_path)
    # cv2.circle(img, (int_x_l, int_y_l), radius=8, color=(255, 0, 0), thickness=-1) 
    # cv2.circle(img, (int_x_r, int_y_r), radius=8, color=(255, 0, 0), thickness=-1) 
    # cv2.imwrite(output_img_path, img)
    # 索引对应关系：5:左肩，6:右肩，11:左臀，12:右臀
    # for kpt_id, (x, y, conf) in enumerate(kpts_data):
    #     if conf > 0.3:  # 只显示置信度大于0.5的关键点
    #         print(f"  关键点 {kpt_id}: 坐标({x:.1f}, {y:.1f}), 置信度{conf:.2f}")


model = YOLO("models/x-pose.pt")
results = model("./img_fix",max_det=1)

result = results[0]

result.save(filename='img_fix_result/hand.jpg')


# #画左右手腕的座标
# kpts_data = result.keypoints.data[0].cpu().numpy()
# left_shoulder = kpts_data[5]
# right_shoulder = kpts_data[6]
# left_hip = kpts_data[11]
# right_hip = kpts_data[12]
# # 4. 计算中点坐标
# # 肩中点 (颈部点)
# shoulder_mid_x = (left_shoulder[0] + right_shoulder[0]) / 2
# shoulder_mid_y = (left_shoulder[1] + right_shoulder[1]) / 2
# shoulder_mid = np.array([shoulder_mid_x, shoulder_mid_y, 
#                             min(left_shoulder[2], right_shoulder[2])])  # 取较低的置信度

# # 髋中点 (腰部中心点)
# hip_mid_x = (left_hip[0] + right_hip[0]) / 2
# hip_mid_y = (left_hip[1] + right_hip[1]) / 2
# hip_mid = np.array([hip_mid_x, hip_mid_y, 
#                     min(left_hip[2], right_hip[2])])  # 取较低的置信度
# #寸的基准,分为两种，分为背部的骨度法和肩部分寸法。
# #第一种背部骨度法
# CUN_SHOULDER_TO_MIDLINE = 18
# distance_pixels_1 = np.sqrt((hip_mid_x - shoulder_mid_x)**2 + 
#                         (hip_mid_y - shoulder_mid_y)**2)
# PIXELS_PER_CUN = distance_pixels_1 / CUN_SHOULDER_TO_MIDLINE
# #第二种肩部分寸法
# CUN_SHOULDER_TO_MIDLINE_2 = 8.5
# distance_pixels_2 = np.sqrt((right_shoulder[0] - shoulder_mid_x)**2 + 
#                         (right_shoulder[1] - shoulder_mid_y)**2)
# PIXELS_PER_CUN_2 = distance_pixels_2 / CUN_SHOULDER_TO_MIDLINE_2

# cun_bi = PIXELS_PER_CUN / PIXELS_PER_CUN_2
# length_bi = distance_pixels_1/distance_pixels_2
# print(f"寸比：{cun_bi}")
# print(f"总长比：{length_bi}")
# for i, r in enumerate(results):
#     # 保存带有关键点标注的图片，文件名为`predict_result.jpg`
#     r.save(filename=f"predict_result_{i}.jpg")
#     print(f"已保存可视化结果图片: predict_result_{i}.jpg")

# # 方法B：以编程方式访问关键点数据（用于你的穴位推算）
# for result in results:
#     # `kpts` 是一个Tensor，形状为 [检测到的人数, 关键点数(17), 维度(3)]
#     # 维度3代表：[x坐标, y坐标, 关键点置信度]
#     kpts = result.keypoints.data
#     print(f"检测到 {len(kpts)} 个人")
    
#     # 遍历每个人
#     for person_id, keypoints in enumerate(kpts):
#         print(f"\n--- 第 {person_id + 1} 个人的关键点 ---")
#         # 遍历每个关键点（COCO 17点格式）
#         # 索引对应关系：5:左肩，6:右肩，11:左臀，12:右臀
#         for kpt_id, (x, y, conf) in enumerate(keypoints):
#             if conf > 0.5:  # 只显示置信度大于0.5的关键点
#                 print(f"  关键点 {kpt_id}: 坐标({x:.1f}, {y:.1f}), 置信度{conf:.2f}")
