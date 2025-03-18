from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    nav2_launch_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py"
    )
    nav2_params = os.path.join(
        os.path.expanduser("~"),
        "ros2_ws",
        "src",
        "wamv_gz",
        "config",
        "wamv_nav2_params.yaml",
    )

    return LaunchDescription(
        [
            # Launch Nav2
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_dir),
                launch_arguments={
                    "params_file": nav2_params,
                    "use_sim_time": "true",
                }.items(),
            ),
            # TF Broadcaster (Odometry)
            Node(
                package="wamv_tf_broadcaster",
                executable="odom_tf_broadcaster",
                name="odom_tf_broadcaster",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            # cmd_vel to Thrusters Node
            Node(
                package="wamv_tf_broadcaster",
                executable="cmd_vel_to_thrusters",
                name="cmd_vel_to_thrusters",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
