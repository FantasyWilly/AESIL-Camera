#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
File   : proxy_server.py
Author : FantasyWilly
Email  : bc697522h04@gmail.com
SPDX-License-Identifier: Apache-2.0

版本:
    • [一般 Python 版本] - lib 庫

開發公司:
    • 先飛科技 (XF)

功能總覽:
    • 啟動 TCP 代理服務，監聽地面端連線
    • 收到命令後，經由持久化的 GCUController
      轉發封包至相機，並將相機回應再傳回地面端

遵循:
    • Google Python Style Guide (含區段標題)
    • PEP 8 (行寬 ≤ 88, snake_case, 2 空行區段分隔)
"""

# ------------------------------------------------------------------------------------ #
# Imports
# ------------------------------------------------------------------------------------ #
# 標準庫
import socketserver


# ------------------------------------------------------------------------------------ #
# ProxyHandler: 處理每一個客戶端連線請求
# ------------------------------------------------------------------------------------ #
class ProxyHandler(socketserver.BaseRequestHandler):
    """
    - 說明 [ProxyHandler]
        1. 接收地面端的 TCP 連線
        2. 讀取收到的 raw bytes 命令
        3. 經由 server.controller(GCUController)透傳至相機
        4. 將相機回應送回地面端
    """

    def handle(self):
        client = self.client_address
        print(f"[代理] 客戶端 {client} 已連線")
        while True:
            try:
                data = self.request.recv(1024)
                if not data:
                    print(f"[代理] 客戶端 {client} 已斷開連線")
                    break

                with self.server.controller.lock:
                    self.server.controller.sock.sendall(data)
                    response = self.server.controller.sock.recv(1024)

                if response:
                    self.request.sendall(response)
                else:
                    print(f"[代理] 客戶端 {client} 未獲得相機回應")

            except ConnectionResetError:
                print(f"[代理] 客戶端 {client} 突然斷開(reset), 退出處理")
                break

            except Exception as e:
                print(f"[代理] 處理連線錯誤：{e}")
                break


# ------------------------------------------------------------------------------------ #
# ThreadedTCPServer: 支援多線程的 TCP 伺服器
# ------------------------------------------------------------------------------------ #
class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    """
    - Allow address reuse so the server can be restarted quickly
    - Each incoming連線由新的執行緒處理, 達到並行轉發
    """
    allow_reuse_address = True


# ------------------------------------------------------------------------------------ #
# ProxyService: 封裝伺服器啟動與關閉邏輯
# ------------------------------------------------------------------------------------ #
class ProxyService:
    """
    - ProxyService 負責：
        1. 建立 ThreadedTCPServer 實例並注入 GCUController
        2. 提供 serve_forever() 啟動伺服器主迴圈
        3. 提供 shutdown() 安全關閉伺服器
    """

    def __init__(self, host: str, port: int, controller):
        self._srv = ThreadedTCPServer((host, port), ProxyHandler)
        self._srv.controller = controller

    def serve_forever(self):
        """啟動伺服器主迴圈，阻塞執行直到 shutdown() 被呼叫"""
        self._srv.serve_forever()

    def shutdown(self):
        """安全關閉伺服器：停止接受新連線並關閉現有連線"""
        self._srv.shutdown()
        self._srv.server_close()
