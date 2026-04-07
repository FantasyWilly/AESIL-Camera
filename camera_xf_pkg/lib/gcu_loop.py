#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : gcu_loop.py  
Author : FantasyWilly   
Email  : bc697522h04@gmail.com  
SPDX-License-Identifier: Apache-2.0 

開發公司:
    • 先飛科技 (XF)

功能總覽:
    • 背景不斷發送空命令, 查詢雲台角度

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
from lib.gcu_controller import GCUController


# ------------------------------------ 循環執行程式 ------------------------------------ #
def loop_in_background(controller: GCUController, stop_event: threading.Event):
    """
    - 說明 [loop_in_background]
        1. 後台執行緒函式，不斷送出維持連線的命令
        2. 當 stop_event 被 set 時 -> 跳出迴圈結束
    """
    while not stop_event.is_set():
        if not rclpy.ok():
            print("[LOOP] rclpy 已關閉，背景迴圈退出")
            break

        try:
            controller.loop_send_command(
                command=0x00,
                parameters=b"",
            )
        except Exception as e:
            if stop_event.is_set() or not rclpy.ok():
                print("[LOOP] 偵測到關閉狀態，背景執行緒結束")
                break

            print(f"[LOOP ERROR] 無法送出資料: {e}")
            time.sleep(0.2)

        time.sleep(0.25)  # 4 Hz

    print("[LOOP] 背景執行緒已結束")
