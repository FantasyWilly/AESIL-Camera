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
# 標準庫
from dataclasses import dataclass

# ROS2
from rclpy.node import Node
from std_msgs.msg import Header
from rclpy.qos import qos_profile_sensor_data

# 專案內部模組
from camera_msgs_pkg.msg import Camera, CameraData, Laser, LaserData


# ------------------------------------------------------------------------------------ #
# 常數 定義
# ------------------------------------------------------------------------------------ #
NODE_NAME = "camera_node"     # Node  節點名稱


# ------------------------------------------------------------------------------------ #
# Config dataclass (讀取參數)
# ------------------------------------------------------------------------------------ #
@dataclass
class CameraConfig:
    # 參數 (target.yaml)
    topic_prefix: str
    camera_topic: str
    laser_topic: str

    @classmethod
    def from_node(cls, node: Node) -> "CameraConfig":
        # 宣告參數
        node.declare_parameter("topic_prefix", "")
        node.declare_parameter("camera_topic", "/camera_data")
        node.declare_parameter("laser_topic", "/laser_data")

        # 讀取參數
        topic_prefix                = node.get_parameter("topic_prefix").value
        camera_suffix               = node.get_parameter("camera_topic").value
        laser_suffix                = node.get_parameter("laser_topic").value

        # 組合 其它 Topic 資訊 (前綴 + 話題名稱)
        def resolve_topic(prefix: str, suffix: str) -> str:
            p = prefix.rstrip("/")
            s = suffix.lstrip("/")
            return f"{p}/{s}"
        
        camera_topic            = resolve_topic(topic_prefix, camera_suffix)
        laser_topic             = resolve_topic(topic_prefix, laser_suffix)

        cfg = cls(
            topic_prefix                = topic_prefix,
            camera_topic                = camera_topic,
            laser_topic                 = laser_topic,
        )

        log = node.get_logger().info
        log(f"[YAML]: target.yaml")
        log(f"[Param] topic_prefix              : {cfg.topic_prefix}")
        log(f"[Param] camera_topic              : {cfg.camera_topic}")
        log(f"[Param] laser_topic               : {cfg.laser_topic}")
        log("------------------------------------------------------")

        return cfg

# ------------------------------------------------------------------------------------ #
# [CUPublisher] 初始化[Node], 宣告參數, 發布相機回傳資訊
# ------------------------------------------------------------------------------------ #
class GCUPublisher(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # 初始化 Node 節點
        self.get_logger().info(f"🔴 [啟動]: {NODE_NAME} 節點")
        self.get_logger().info("------------------------------------------------------")

        self.cfg = CameraConfig.from_node(self)

        # 建立 ROS2 publishers
        self.publisher_camera = self.create_publisher(
            Camera, 
            self.cfg.camera_topic, 
            qos_profile_sensor_data
        )

        # 建立 ROS2 publishers
        self.publisher_laser = self.create_publisher(
            Laser, 
            self.cfg.laser_topic, 
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


