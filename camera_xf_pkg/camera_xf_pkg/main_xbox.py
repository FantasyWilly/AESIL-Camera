#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : main_xbox.py
Author : FantasyWilly   
Email  : bc697522h04@gmail.com  
SPDX-License-Identifier: Apache-2.0 

版本:
    • [ROS2 版本]

開發公司:
    • 先飛科技 (XF)

功能總覽:
    • Xbox 搖桿控制
    • 發送相機控制指令
    • 接收相機回傳資料
    • 回傳資料發布至 ROS2
"""

# ------------------------------------------------------------------------------------ #
# Imports
# ------------------------------------------------------------------------------------ #
# 標準庫
import time
import pygame
import threading

# ROS2
import rclpy

# 專案內部模組
import lib.ros.camera_command as cm
import lib.ros.gcu_loop as gcu_loop
from lib.ros.gcu_controller import GCUController


# ------------------------------------------------------------------------------------ #
# TCP 連線 <IP:Port> 
# ------------------------------------------------------------------------------------ #
DEVICE_IP = "192.168.144.108"
DEVICE_PORT = 2332


# ------------------------------------------------------------------------------------ #
# 每次 Gimbal 每步移動度數
# ------------------------------------------------------------------------------------ #
CONTROL_INCREMENT = 5.0


# ------------------------------------------------------------------------------------ #
# xbox 傳輸控制指令
# ------------------------------------------------------------------------------------ #
def xbox_controller_loop(controller, stop_event: threading.Event):

    # 初始化 xbox 搖桿
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("未找到 Xbox 控制器")
        return

    # 取得第一個連接的控制器並初始化
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Xbox 控制器已啟動")


    while True:
        for event in pygame.event.get():

            # 按鍵 - [A, B, X, Y, L, R]    
            if event.type == pygame.JOYBUTTONDOWN:
                if joystick.get_button(0):
                    print("A 按鈕按下: 向下")
                    cm.down(controller)
                elif joystick.get_button(1):
                    print("B 按鈕按下: 拍照")
                    cm.photo(controller)
                elif joystick.get_button(2):
                    print("X 按鈕按下: 錄影")
                    cm.video(controller)
                elif joystick.get_button(3):
                    print("Y 按鈕按下: 回中")
                    cm.reset(controller)
                elif joystick.get_button(4):
                    print("L 按鈕按下: 鎖頭")
                    cm.lock(controller)
                elif joystick.get_button(5):
                    print("R 按鈕按下: 跟隨")
                    cm.follow(controller)

            # 按鍵 - [上下左右]    
            elif event.type == pygame.JOYHATMOTION:
                hat = joystick.get_hat(0)
                pitch = hat[1] * CONTROL_INCREMENT
                yaw   = hat[0] * CONTROL_INCREMENT
                if hat != (0, 0):
                    print(f"發送雲台控制指令 -> pitch: {pitch}°, yaw: {yaw}°")
                    cm.control_gimbal(controller, pitch=pitch, yaw=yaw)

            # 按鍵 - [右扳機 (RT) - 5], [左扳機 (LT) - 2]
            elif event.type == pygame.JOYAXISMOTION:

                if event.axis == 5:
                    rt_value = joystick.get_axis(5)
                    if rt_value > 0.5:
                        print("RT 按下: zoom_in")
                        cm.zoom_in(controller)
                    else:
                        print("RT 釋放: zoom_stop")
                        cm.zoom_stop(controller)
                
                elif event.axis == 2:
                    lt_value = joystick.get_axis(2)
                    if lt_value > 0.5:
                        print("LT 按下: zoom_out")
                        cm.zoom_out(controller)
                    else:
                        print("LT 釋放: zoom_stop")
                        cm.zoom_stop(controller)
        time.sleep(0.1)


# ------------------------------------------------------------------------------------ #
# 主程式
# ------------------------------------------------------------------------------------ #
def main():
    """
    - 說明 [main]
        1. 創建 [GCUController] 並 連線至 GCU控制盒
        2. 初始化 ROS2
        3. 連續發送空命令 -> 接收返回資訊
        4. Xbox 搖桿控制
    """

    # 初始化 ROS2 
    rclpy.init()

    # 建立 TCP 連線物件 - [GCUController]
    controller = GCUController(DEVICE_IP, DEVICE_PORT)

    try:
        # 1. TCP 連線
        controller.connect()

        # 2-1. 建立一個 stop_event 讓背景線程知道什麼時候要結束
        stop_event = threading.Event()

        # 2-2. 開啟 [LOOP] 背景線程, 持續發送空命令
        loop_thread = threading.Thread(
            target=gcu_loop.loop_in_background,
            args=(controller, stop_event),
            daemon=True
        )
        loop_thread.start()
        print("[LOOP] - 開始不斷發送空命令")

        # 3-1. 開啟 Xbox 遙控控制
        xbox_thread = threading.Thread(
            target=xbox_controller_loop,
            args=(controller, stop_event),
            daemon=True
        )
        xbox_thread.start()

        # 主线程阻塞
        while rclpy.ok() and not stop_event.is_set():
            time.sleep(0.2)

    except Exception as e:
        print("[main] 出現錯誤:", e)
        stop_event.set()
    finally:
        # 通知所有线程退出
        stop_event.set()
        loop_thread.join()
        xbox_thread.join()
        controller.disconnect()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
