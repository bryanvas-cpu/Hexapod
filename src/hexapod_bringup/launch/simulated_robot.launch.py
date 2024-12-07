import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("hexapod_description"),
            "launch",
            "gazebo.launch.py"
        )
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
        )
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("hexapod_description"),
                "rviz",
                "hexapod.rviz"
            )
        ],
        output="screen",
    )


    
    return LaunchDescription([
        gazebo,
        controller,
        joystick,
        rviz,
        # group
    ])