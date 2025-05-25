from setuptools import setup, find_packages

package_name = 'puzzlebot_line_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Para que ROS2 encuentre el package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Para instalar package.xml
        ('share/' + package_name, ['package.xml']),
        # Params y launch
        ('share/' + package_name + '/params',
            ['params/line_follower_params.yaml']),
        ('share/' + package_name + '/launch',
            ['launch/line_follow.launch.py']),
        # Modelos (incluimos tu .pt para YOLOv8)
        ('share/' + package_name + '/models',
            ['models/lastfinal.pt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jp',
    maintainer_email='jp@todo.todo',
    description='Line follower for Puzzlebot using ROS 2, OpenCV and YOLOv8',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'line_light_detector = puzzlebot_line_follower.line_light_detector:main',
            'line_follower_controller = puzzlebot_line_follower.line_follower_controller:main',
            'yolov8_detector = puzzlebot_line_follower.yolov8_detector:main',
        ],
    },
)
