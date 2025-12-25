# target_laser_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
import os

def generate_launch_description():

    pkg = "camera_xf_pkg"
    share = get_package_share_directory(pkg)

    camera_cfg = os.path.join(share, "config", "camera.yaml")

    return LaunchDescription([

        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),

        Node(
            package=pkg,
            executable="main_xbox",
            name="camera_node",
            output="screen",
            parameters=[camera_cfg]
        )

    ])

