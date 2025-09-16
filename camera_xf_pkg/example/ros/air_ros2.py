#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : air_ros2.py
Author : FantasyWilly   
Email  : bc697522h04@gmail.com  
SPDX-License-Identifier: Apache-2.0 

版本:
    • [ROS2 版本]

開發公司:
    • 先飛科技 (XF)

功能總覽:
    • 啟動一個 TCP 代理服務, 監聽連線端口
    • 每次接收到命令時, 直接利用 controller 持久連線 轉發命令給相機

遵循:
    • Google Python Style Guide (含區段標題)
    • PEP 8 (行寬 ≤ 88, snake_case, 2 空行區段分隔)
"""


# ------------------------------------------------------------------------------------ #
# Imports
# ------------------------------------------------------------------------------------ #
# 標準庫
import time
import threading

# ROS2
import rclpy

# 專案內部模組
from lib.ros.gcu_loop import loop_in_background
from lib.ros.gcu_controller import GCUController
from lib.ros.proxy_server import ProxyService


# ------------------------------------------------------------------------------------ #
# TCP 連線 <IP:Port> 
# ------------------------------------------------------------------------------------ #
DEVICE_IP = "192.168.144.108"       # 相機 IP
DEVICE_PORT = 2332                  # 相機埠號

PROXY_LISTEN_IP = "0.0.0.0"         # 代理服務監聽的 IP
PROXY_LISTEN_PORT = 9999            # 代理服務監聽的埠號


# ------------------------------------------------------------------------------------ #
# 主程式
# ------------------------------------------------------------------------------------ #
def main():

    # 初始化 ROS2 
    rclpy.init()

    # 建立 TCP 連線物件 - [GCUController]
    controller = GCUController(DEVICE_IP, DEVICE_PORT)

    # 1. TCP 連線
    controller.connect()

    # 2-1. 建立一個 stop_event 讓背景線程知道什麼時候要結束
    stop_event = threading.Event()

    # 2-2. 開啟 [LOOP] 背景線程, 持續發送空命令
    loop_thread = threading.Thread(
        target=loop_in_background,
        args=(controller, stop_event),
        daemon=True
    )
    loop_thread.start()
    print("[LOOP] - 開始不斷發送空命令")

    # 3. 啟動代理服務器
    proxy = ProxyService('0.0.0.0', 9999, controller)
    proxy_thread = threading.Thread(
        target=proxy.serve_forever,
        daemon=True
    )
    proxy_thread.start()
    print("[PROXY] - 代理服務器已啟動")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_event.set()
        loop_thread.join()
        proxy.shutdown()
        proxy_thread.join()
        controller.disconnect()
        rclpy.shutdown()

if __name__ == "__main__":
    main()