# <div align="center">AESIL - Camera</div>

## <div align="center">Outline</div>

- [程式下載 (Downloads)](#downloads)
- [相機控制 (Camera Controll)](#camera)

## <div align="center">Downloads</div>

*Step1* - 下載至 ROS2 的工作空間
```bash
cd ~/<your_workspace>/src
git clone https://github.com/FantasyWilly/AESIL-Camera.git
```
---

*Step2* - 編譯工作空間
```bash
cd ~/<your_workspace>
colcon build
source ~/.bashrc
```
---

## <div align="center">Camera</div>

  ### [ XF 系列 ]

  - `官網連結:` [ 先飛科技 ](https://www.allxianfei.com/) - [使用手冊 控制協議 3D圖檔等...]

  - ROS2 Running (基礎使用)

    ```bash
    ros2 run camera_xf_pkg main_cmd         # CMD  指令輸入控制
    ros2 run camera_xf_pkg main_xbox        # Xbox 搖桿輸入控制
    ```

    `Node:` /gcu_publisher_node
  
    `Topic:` /camera_data_pub
    
    ---

  - `補充:` 發送端 須下載 `AESIL-CameraExtra` 功能包 --- [跳轉連結](https://github.com/FantasyWilly/AESIL-CameraExtra)

  - ⚠️ 記得修改 發送端 檔案連接的 `Server` IP, Port 且 在同網域底下

    ### 載具端
    ```bash
    # 接收控制指令 - [監聽端]
    ros2 run camera_xf_pkg main_air              
    ```

    ----

    ### 發送端
    ```bash
    # 發送控制指令 - [發送端 (Xbox連接控制)]
    python3 ~/<你的路徑>/AESIL-CameraExtra/camera_ground/XF/main_ground_xbox.py  
    ```

  ---


  