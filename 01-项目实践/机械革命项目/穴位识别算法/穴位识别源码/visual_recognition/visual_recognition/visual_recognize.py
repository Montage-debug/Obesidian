#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
视觉识别节点
"""

# 导出 main 函数，防止 PyArmor RFT 模式重命名（entry_point 需要引用）
__all__ = ['main', 'VisualRecognizeNode']

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import VisualAcupointRecognize
from robot_interfaces.msg import AcupointInfo, ModuleStates
from std_msgs.msg import Header
import base64
import cv2
import numpy as np
from .draw_point import *

class VisualRecognizeNode(Node):
    """视觉识别节点类"""

    def __init__(self):
        super().__init__('visual_recognize')


        # 创建视觉识别服务
        self.service = self.create_service(
            VisualAcupointRecognize,
            '/visual/recognize_acupoint',
            self.recognize_acupoint_callback
        )

        # 创建节点状态心跳发布者（1Hz），供系统监控判断进程存活
        self._states_pub = self.create_publisher(ModuleStates, '/visual_recognize/states', 10)
        self._states_timer = self.create_timer(1.0, self._publish_states)

        self.get_logger().info('视觉识别节点已启动')
        self.get_logger().info('视觉识别服务已就绪: /visual/recognize_acupoint')

    def _publish_states(self):
        """每秒发布一次节点心跳，state_json 预留为空 JSON 供后续扩展"""
        msg = ModuleStates()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        msg.state_json = '{}'  # 当前仅作存活心跳，无额外字段
        self._states_pub.publish(msg)

    def recognize_acupoint_callback(self, request, response):
        """
        视觉识别服务回调函数
        
        Args:
            request: VisualAcupointRecognize.Request
                - image_base64: 图片的base64编码
                - body_part_code: 识别的部位代码
            response: VisualAcupointRecognize.Response
                - return_code: 返回码
                - return_msg: 返回消息
                - acupoints_data_json: 识别结果的json格式数据
                
        Returns:
            VisualAcupointRecognize.Response
        """
        start_time = time.time()
        self.get_logger().info(f'收到识别请求 - 部位代码: {request.body_part_code}')
        
        try:
            # 将 base64 编码的图片转换为 jpg 格式
            image_data = base64.b64decode(request.image_base64)
            # 将字节数据转换为 numpy 数组
            nparr = np.frombuffer(image_data, np.uint8)
            # 使用 OpenCV 解码图像为 jpg 格式
            image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if image is None:
                response.return_code = -99
                response.return_msg = '无法解码图像数据'
                raise ValueError("无法解码图像数据")
            
            self.get_logger().info(f'成功解码图像 - 尺寸: {image.shape}')
            
            success, result_code, message, acupoints_to_draw, shoulder_width, hip_width, img_with_points = chose_body_part(
                image,
                body_part_code=request.body_part_code,
                output_path="body_with_landmark.jpg"
                )
            if result_code < 0:
                result_code = -1  #将底层所有识别异常的错误码统一为-1，给上层。
            
            cv2.imwrite("orig_img.jpg", image)
            self.get_logger().info(f'识别算法返回 - success: {success}, result_code: {result_code}, message: {message}')
            
            # 检查返回值是否有效
            if acupoints_to_draw is None:
                self.get_logger().warning('识别算法返回的穴位列表为 None，使用空列表')
                acupoints_to_draw = []

            # 构建穴位列表
            acupoints_list = []
            for acupointCode, acupoint_name, coord in acupoints_to_draw:
                x, y = coord
                acupoint_info = AcupointInfo()
                acupoint_info.acupoint_code = acupointCode
                acupoint_info.acupoint_name = acupoint_name
                acupoint_info.pixel_x = float(x)
                acupoint_info.pixel_y = float(y)
                acupoints_list.append(acupoint_info)
            
            self.get_logger().info(f'识别到 {len(acupoints_list)} 个穴位')
            
            # 返回成功响应（result_code 为 0 时透传，负数时偏移 -1）
            response.return_code = result_code
            
            response.return_msg = message
            response.acupoints_data_list = acupoints_list
            response.shoulder_width = shoulder_width
            response.hip_width = hip_width
            
            self.get_logger().info(f'识别完成 - 部位: {request.body_part_code}')
            
        except Exception as e:
            # 处理异常
            response.return_code = -13
            response.return_msg = '识别算法调用异常'
            self.get_logger().error(f'识别失败: {str(e)}')
        end_time = time.time()
        self.get_logger().info(f'识别用时: {end_time - start_time:.2f} 秒')
        
        return response


def main(args=None):
    """主函数"""
    # 授权验证
    try:
        from license_validator_py import check_system_license
        license_result = check_system_license()
        if not license_result.is_valid:
            print(f'[VisualRecognizeNode] 授权验证失败: {license_result.message}')
            return
    except ImportError as e:
        print(f'[VisualRecognizeNode] 无法加载授权验证模块: {e}')
        return
    except Exception as e:
        print(f'[VisualRecognizeNode] 授权验证异常: {e}')
        return

    rclpy.init(args=args)
    node = VisualRecognizeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
