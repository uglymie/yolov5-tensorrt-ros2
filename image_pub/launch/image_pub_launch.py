from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('cpp_advanced_lane_lines')

    return LaunchDescription([
        Node(
            package='image_pub',
            executable='image_pub',
            parameters=[
                {"image_pub_topic_name": '/camera/image_raw'},
                {"capture_format": '/data/road_video/project_video.mp4'},
                
                # {"capture_format": '0'}
            ]
        )
 
    ])
