#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : cmd.py  
Author : FantasyWilly   
Email  : bc697522h04@gmail.com  
SPDX-License-Identifier: Apache-2.0 

版本:
    • [一般 Python 版本]

開發公司:
    • 先飛科技 (XF)

功能總覽:
    • CMD 鍵盤輸入控制
    • 發送相機控制指令
    • 接收相機回傳資料

遵循:
    • Google Python Style Guide (含區段標題)
    • PEP 8 (行寬 ≤ 88, snake_case, 2 空行區段分隔)
"""

# ------------------------------------------------------------------------------------ #
# Imports
# ------------------------------------------------------------------------------------ #
# 標準庫
import threading

# 專案內部模組
import lib.common.gcu_loop as gcu_loop
import lib.common.camera_command as cm
from lib.common.gcu_controller import GCUController


# ------------------------------------------------------------------------------------ #
# TCP 連線 <IP:Port> 
# ------------------------------------------------------------------------------------ #
DEVICE_IP = "192.168.144.108"
DEVICE_PORT = 2332


# ------------------------------------------------------------------------------------ #
# 主程式
# ------------------------------------------------------------------------------------ #
def main():
    """
    - 說明 [main]
        1. 創建 [GCUController] 並 連線至 GCU控制盒
        2. 連續發送空命令 -> 接收返回資訊
        3. 讓使用者輸入指令
    """

    # 建立 TCP 連線物件 - [GCUController]
    controller = GCUController(DEVICE_IP, DEVICE_PORT)

    try:
        # 1. TCP 連線
        controller.connect()

        # 2-1. 建立一個 stop_event 讓背景線程知道什麼時候要結束
        stop_event = threading.Event()

        # 2-2. 開啟 [LOOP] 背景線程
        loop_thread = threading.Thread(
            target=gcu_loop.loop_in_background,
            args=(controller, stop_event),
            daemon=True
        )
        loop_thread.start()
        print("[LOOP] - 開始不斷發送空命令")

        # 3. 創建 CMD 輸入控制指令界面
        while True:
            cmd = input(
                "请输入指令 "
                "(empty / reset / down /photo / video / quit): "
            ).strip().lower()

            if cmd == 'empty':
                cm.empty(controller)
            elif cmd == 'reset':
                cm.reset(controller)
            elif cmd == "down":
                cm.down(controller)
            elif cmd == "photo":
                cm.photo(controller)
            elif cmd == "video":
                cm.video(controller)
            elif cmd == "quit":
                print("已退出操作")
                break
            else:
                print("無效指令,請重新輸入")

    except Exception as e:
        print("[main] 出現錯誤:", e)
    finally:
        controller.disconnect()

if __name__ == "__main__":
    main()
