#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Ruta a los parámetros del controlador
    pkg_share = get_package_share_directory('puzzlebot_line_follower')
    lf_params = os.path.join(pkg_share, 'params', 'line_follower_params.yaml')

    return LaunchDescription([
        # Nodo de detección de línea
        Node(
            package='puzzlebot_line_follower',
            executable='line_light_detector',  # definido en line_light_detector.py :contentReference[oaicite:0]{index=0}
            name='line_light_detector',
            output='screen',
        ),

        # Nodo de control de seguimiento
        Node(
            package='puzzlebot_line_follower',
            executable='line_follower_controller',  # definido en line_follower_controller.py :contentReference[oaicite:1]{index=1}
            name='line_follower_controller',
            output='screen',
            parameters=[lf_params],
        ),
    ])
