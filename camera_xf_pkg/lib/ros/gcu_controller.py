#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : gcu_controller.py  
Author : FantasyWilly   
Email  : bc697522h04@gmail.com  
SPDX-License-Identifier: Apache-2.0 

版本:
    • [ROS2 版本] - lib 庫

開發公司:
    • 先飛科技 (XF)

功能總覽:
    • 管理TCP連線
    • 發送控制命令
    • 連續發送空命令 & 解碼並發布至ROS2

遵循:
    • Google Python Style Guide (含區段標題)
    • PEP 8 (行寬 ≤ 88, snake_case, 2 空行區段分隔)
"""

# ------------------------------------------------------------------------------------ #
# Imports
# ------------------------------------------------------------------------------------ #
# 標準庫
import socket
import threading

# 專案內部模組
from lib.ros.gcu_publisher import GCUPublisher
from lib.ros.camera_protocol import build_packet
from lib.ros.camera_decoder import decode_gcu_response


# ------------------------------------------------------------------------------------ #
# [GCUController] 用於連接 發送指令和接收響應
# ------------------------------------------------------------------------------------ #
class GCUController:
    """
    - 說明 [GCUController]
        1. 接收 IP, Port 參數
        2. 管理 TCP 連線
        3. 發送 控制命令

    args:
        • ip (str)                      - 目標主機 IP
        • port (int)                    - 目標主機 Port
        • timeout (float)      [Optional]    - Socket 超時時間      (default: 5s)
        • ros2_publisher_node  [Optional]    - 傳入 ROS2 發布者節點  (default: None)
    """

    def __init__(
        self, ip: str, 
        port: int, 
        timeout: float = 5.0, 
    ) -> None:
        
        # 接收參數
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)

        # 創建 ROS2 Publisher物件
        self.ros2_publisher_node = GCUPublisher()

        # 保護整個 send/recv 流程
        self.lock = threading.Lock()

    # ------------------------ (connect) 開啟 TCP連接 --------------------------------- #
    def connect(self) -> None:
        self.sock.connect((self.ip, self.port))
        print(f"已連接到 GCU: {self.ip}:{self.port}")

    # ----------------------- (disconnect) 關閉 TCP連接 ------------------------------- #
    def disconnect(self) -> None:
        self.sock.close()
        print("連接已關閉")

    # ---------------------- (send_command) 發送 控制命令 ------------------------------ #
    def send_command(
        self, 
        command: int, 
        parameters: bytes = b'', 
        enable_request: bool = False,
        pitch: float = None, yaw: float = None,
        x0: int = None, y0: int = None, x1: int = None, y1: int = None
    ) -> bytes:
        
        """
        args:
            • command (int)         - 16 進位
            • parameters (bytes)    - 16 進位 
            • enable_request (bool) - 是否須返回 GCU 數據格式   (default: True)
            • pitch, yaw (float)    - 控制台角度               (default: None) 
            • x0, y0, x1, y1 (int)  - 框選方框四角點            (default: None)

        returns:
            • response (bytes)      - 返回 GCU 數據格式
        """

        with self.lock:
            
            # 1. 構建數據包並發送
            packet = build_packet(
                command,
                parameters,
                enable_request,
                pitch=pitch, yaw=yaw,
                x0=x0, y0=y0, x1=x1, y1=y1
            )
            # print("發送 [數據包] :", packet.hex().upper())
            self.sock.sendall(packet)

            # 2. 接收本次指令的回覆
            response = self.sock.recv(256)
            # print("接收 [返回數據] :", response.hex().upper())

        # 3. 解碼本次指令回覆
        parsed = decode_gcu_response(response)
        if 'error' in parsed:
            print("解碼失敗:", parsed['error'])
        else:
            roll = parsed['roll']
            pitch = parsed['pitch']
            yaw = parsed['yaw']
            ratio = parsed['ratio']

            # 傳送資訊至 ROS2
            self.ros2_publisher_node.publish_camera_data(roll, pitch, yaw, ratio)

        return response

    # ---------------------------------  不斷 發送空命令 ------------------------------- #
    def loop_send_command(
        self, 
        command: int, 
        parameters: bytes = b'', 
        enable_request: bool = True,
    ) -> bytes:
        
        with self.lock:

            # 1. 構建數據包並發送
            packet = build_packet(
                command, 
                parameters, 
                enable_request
            )
            # print("發送 [數據包] :", packet.hex().upper())
            self.sock.sendall(packet)

            # 2. 接收本次指令的回覆
            response = self.sock.recv(256)
            # print("接收 [返回數據] :", response.hex().upper())

        # 3. 解碼本次指令回覆
        parsed = decode_gcu_response(response)
        if 'error' in parsed:
            print("解碼失敗:", parsed['error'])
        else:
            roll = parsed['roll']
            pitch = parsed['pitch']
            yaw = parsed['yaw']
            ratio = parsed['ratio']

            # 傳送資訊至 ROS2
            self.ros2_publisher_node.publish_camera_data(roll, pitch, yaw,ratio)

        return response
