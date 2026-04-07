#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : main_air.py
Author : FantasyWilly
Email  : bc697522h04@gmail.com
SPDX-License-Identifier: Apache-2.0

開發公司:
    • 先飛科技 (XF)

功能總覽:
    • 啟動一個 TCP 代理服務, 監聽連線端口
    • 每次接收到命令時, 直接利用 controller 持久連線轉發命令給相機
    • 可被 systemd 正常停止, 支援 SIGTERM / SIGINT 優雅關閉

遵循:
    • Google Python Style Guide (含區段標題)
    • PEP 8 (行寬 ≤ 88, snake_case, 2 空行區段分隔)
"""

# ------------------------------------------------------------------------------------ #
# Imports
# ------------------------------------------------------------------------------------ #
# 標準庫
import signal
import threading
import time

# 第三方套件
import cv2

# ROS2
import rclpy

# 專案內部模組
import lib.camera_command as cm
from lib.gcu_controller import GCUController
from lib.gcu_loop import loop_in_background
from lib.proxy_server import ProxyService


# ------------------------------------------------------------------------------------ #
# TCP 連線 <IP:Port>
# ------------------------------------------------------------------------------------ #
DEVICE_IP = "192.168.144.108"       # 相機 IP
DEVICE_PORT = 2332                  # 相機埠號

PROXY_LISTEN_IP = "0.0.0.0"         # 代理服務監聽的 IP
PROXY_LISTEN_PORT = 9999            # 代理服務監聽的埠號


# ------------------------------------------------------------------------------------ #
# 影像串流 <CAMERA_URL>
# ------------------------------------------------------------------------------------ #
CAMERA_URL = "rtsp://192.168.144.108"


# ------------------------------------------------------------------------------------ #
# Global Shutdown Event
# ------------------------------------------------------------------------------------ #
shutdown_event = threading.Event()


# ------------------------------------------------------------------------------------ #
# Signal Handler
# ------------------------------------------------------------------------------------ #
def handle_shutdown_signal(signum, frame):
    """接收 systemd 或使用者送來的終止訊號。"""
    del frame
    print(f"[SIGNAL] 收到終止訊號: {signum}")
    shutdown_event.set()


# ------------------------------------------------------------------------------------ #
# Main
# ------------------------------------------------------------------------------------ #
def main():
    """主程式進入點。"""
    cap = None
    controller = None
    stop_event = None
    loop_thread = None
    proxy = None
    proxy_thread = None
    ros_initialized = False

    # 註冊系統訊號，讓 systemctl stop 能正常關閉
    signal.signal(signal.SIGTERM, handle_shutdown_signal)
    signal.signal(signal.SIGINT, handle_shutdown_signal)

    try:
        # --------------------------------------------------
        # 影像串流解析
        # --------------------------------------------------
        print("[INIT] 正在檢查影像串流...")
        cap = cv2.VideoCapture(CAMERA_URL)

        if not cap.isOpened():
            print(f"[CAMERA_URL] 無法連接到串流: {CAMERA_URL}")
            width = 0
            height = 0
        else:
            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"[CAMERA_URL] 畫面大小: {width}x{height}")

            cap.release()
            cap = None

        # --------------------------------------------------
        # ROS2 Init
        # --------------------------------------------------
        print("[ROS2] 初始化中...")
        rclpy.init()
        ros_initialized = True
        print("[ROS2] 初始化完成")

        # --------------------------------------------------
        # Controller
        # --------------------------------------------------
        print("[CONTROLLER] 建立控制器...")
        controller = GCUController(DEVICE_IP, DEVICE_PORT, width, height)
        controller.connect()
        print("[CONTROLLER] 已連線")

        # --------------------------------------------------
        # Background Loop
        # --------------------------------------------------
        stop_event = threading.Event()
        loop_thread = threading.Thread(
            target=loop_in_background,
            args=(controller, stop_event),
            daemon=True,
            name="gcu_loop_thread",
        )
        loop_thread.start()
        print("[LOOP] 開始不斷發送空命令")

        # --------------------------------------------------
        # Proxy Service
        # --------------------------------------------------
        proxy = ProxyService(PROXY_LISTEN_IP, PROXY_LISTEN_PORT, controller)
        proxy_thread = threading.Thread(
            target=proxy.serve_forever,
            daemon=True,
            name="proxy_service_thread",
        )
        proxy_thread.start()
        print(
            f"[PROXY] 代理服務器已啟動: "
            f"{PROXY_LISTEN_IP}:{PROXY_LISTEN_PORT}"
        )

        # --------------------------------------------------
        # 初始命令
        # --------------------------------------------------
        # try:
        #     cm.down(controller)
        # except Exception as exc:
        #     print(f"[WARN] 初始 down 命令失敗: {exc}")

        # --------------------------------------------------
        # Main Loop
        # --------------------------------------------------
        print("[MAIN] 進入主迴圈，等待停止訊號...")
        while not shutdown_event.is_set():
            time.sleep(0.2)

    except KeyboardInterrupt:
        print("[MAIN] 收到 KeyboardInterrupt")
        shutdown_event.set()

    except Exception as exc:
        print(f"[ERROR] main_air 發生例外: {exc}")
        shutdown_event.set()

    finally:
        print("[CLEANUP] 正在釋放資源...")

        # --------------------------------------------------
        # 通知背景執行緒停止
        # --------------------------------------------------
        shutdown_event.set()

        if stop_event is not None:
            stop_event.set()
            print("[CLEANUP] 已通知 loop thread 停止")

        # --------------------------------------------------
        # 關閉 Proxy Service
        # --------------------------------------------------
        if proxy is not None:
            try:
                proxy.shutdown()
                print("[CLEANUP] Proxy Service shutdown 完成")
            except Exception as exc:
                print(f"[WARN] Proxy Service shutdown 失敗: {exc}")

        # --------------------------------------------------
        # 等待背景執行緒結束
        # --------------------------------------------------
        if loop_thread is not None and loop_thread.is_alive():
            print("[CLEANUP] 等待 loop thread 結束...")
            loop_thread.join(timeout=5)
            if loop_thread.is_alive():
                print("[WARN] loop thread 尚未完全結束")
            else:
                print("[CLEANUP] loop thread 已結束")

        if proxy_thread is not None and proxy_thread.is_alive():
            print("[CLEANUP] 等待 proxy thread 結束...")
            proxy_thread.join(timeout=5)
            if proxy_thread.is_alive():
                print("[WARN] proxy thread 尚未完全結束")
            else:
                print("[CLEANUP] proxy thread 已結束")

        # --------------------------------------------------
        # 中斷 Controller 連線
        # --------------------------------------------------
        if controller is not None:
            try:
                controller.disconnect()
                print("[CLEANUP] Controller 已斷線")
            except Exception as exc:
                print(f"[WARN] Controller disconnect 失敗: {exc}")

        # --------------------------------------------------
        # 釋放影像資源
        # --------------------------------------------------
        if cap is not None:
            try:
                cap.release()
                print("[CLEANUP] VideoCapture 已釋放")
            except Exception as exc:
                print(f"[WARN] VideoCapture 釋放失敗: {exc}")

        # --------------------------------------------------
        # 最後才關閉 ROS2
        # --------------------------------------------------
        if ros_initialized:
            try:
                if rclpy.ok():
                    rclpy.shutdown()
                    print("[CLEANUP] ROS2 已 shutdown")
            except Exception as exc:
                print(f"[WARN] ROS2 shutdown 失敗: {exc}")

        print("[CLEANUP] 完成")


if __name__ == "__main__":
    main()