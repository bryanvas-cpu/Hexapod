import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import Node


def generate_launch_description():
    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("hexapod_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("hexapod_controller"),
            "launch",
            "controller.launch.py"
        )
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("hexapod_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
    )

    imu_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("mpu9250"),
                "launch",
                "mpu9250.launch.py")
        ])
    )
    
    return LaunchDescription([
        hardware_interface,
        controller,
        joystick,
        # group,
        # imu_driver
    ])