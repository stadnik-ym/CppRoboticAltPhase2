from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
#     camera_path_arg = DeclareLaunchArgument(
#         'camera_path',
#         default_value='0',
#         description='Camera device index or path'
#     )

    aruco_detector_node = Node(
        package='aruco_detector',
        executable='aruco_detector_node',
        name='aruco_detector',
        output='screen',
        emulate_tty=True,
        respawn=True,
    )

    camera_publisher_node = Node(
        package='aruco_detector',
        executable='camera_publisher_node',
        name='camera_publisher_node',
        output='screen',
        emulate_tty=True,
        # parameters=[{'camera_path': LaunchConfiguration('camera_path')}],
        parameters = [],
    )

    return LaunchDescription([aruco_detector_node, camera_publisher_node])
