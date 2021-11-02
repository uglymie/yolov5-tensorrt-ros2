from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('yolov5_tensorrt')

    yaml_path = pkg_share_dir + '/configs/config.yaml'

    return LaunchDescription([
        Node(
            package='image_pub',
            executable='image_pub',
            parameters=[
                {"image_pub_topic_name": '/camera/image_raw'},
                # {"capture_format": '/data/road_video/project_video.mp4'}
                {"capture_format": '0'}  # USB摄像头
            ]
        ),

        Node(
            package='yolov5_tensorrt',
            executable='yolov5_tensorrt',
            name='yolov5_tensorrt',
            parameters=[
                {"engine_dir": pkg_share_dir + '/weights/yolov5x.engine'},
                yaml_path,
            ],
        )
    ])
