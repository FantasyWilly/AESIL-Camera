#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : gcu_publisher.py
Author : FantasyWilly   
Email  : bc697522h04@gmail.com  
SPDX-License-Identifier: Apache-2.0 

開發公司:
    • 先飛科技 (XF)

功能總覽:
    • 接收解碼數據
    • 解碼數據發布至 ROS2

遵循:
    • Google Python Style Guide (含區段標題)
    • PEP 8 (行寬 ≤ 88, snake_case, 2 空行區段分隔)
"""

# ------------------------------------------------------------------------------------ #
# Import
# ------------------------------------------------------------------------------------ #
# ROS2
from rclpy.node import Node
from std_msgs.msg import Header
from rclpy.qos import qos_profile_sensor_data

# 專案內部模組
from camera_msgs_pkg.msg import Camera, CameraData, Laser, LaserData


# ------------------------------------------------------------------------------------ #
# [CUPublisher] 初始化[Node], 宣告參數, 發布相機回傳資訊
# ------------------------------------------------------------------------------------ #
class GCUPublisher(Node):
    def __init__(self):
        
        # 初始化 Node 節點
        super().__init__('gcu_publisher_node')

        # 建立 ROS2 publishers
        self.publisher_camera = self.create_publisher(
            Camera, 
            '/camera_data', 
            qos_profile_sensor_data
        )

        # 建立 ROS2 publishers
        self.publisher_laser = self.create_publisher(
            Laser, 
            '/laser_data', 
            qos_profile_sensor_data
        )

    # -------------------------- 接收 Camera 資料 & 發布至ROS2 ------------------------- #
    def publish_camera_data(
        self, 
        roll: float, pitch: float, yaw: float, 
        zoom: float, 
        targetdist: float
    ) -> None:
        
        # Camera 相關訊息
        camera_data = CameraData()
        camera_data.rollangle = roll
        camera_data.yawangle = yaw
        camera_data.pitchangle = pitch
        camera_data.zoom = zoom
        
        camera_msg = Camera()
        camera_msg.header = Header(stamp=self.get_clock().now().to_msg())
        camera_msg.data = [camera_data]

        self.publisher_camera.publish(camera_msg)
        # self.get_logger().info(
        #     "[發布] Camera 資料: "
        #     f"[ROLL]={camera_data.rollangle:.2f}, "
        #     f"[YAW]={camera_data.yawangle:.2f}, "
        #     f"[PITCH]={camera_data.pitchangle:.2f}"
        #     f"[RATIO]={camera_data.zoom_ratio:.2f}"
        # )

        # Laser 雷射測距消息
        laser_data = LaserData()
        laser_data.targetdist = targetdist

        laser_msg = Laser()
        laser_msg.header = Header(stamp=self.get_clock().now().to_msg())
        laser_msg.data = [laser_data]

        self.publisher_laser.publish(laser_msg)
        # self.get_logger().info(
        #     "[發布] Laser 資料: "
        #     f"[TARGETDIST]={laser_data.targetdist:.1f},"
        # )


