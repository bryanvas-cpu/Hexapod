from launch import LaunchDescription
from launch.actions import TimerAction, GroupAction
from launch_ros.actions import Node

def generate_launch_description():
    group = GroupAction(
        actions=[
            Node(
                package="hexapod_controller",
                executable="inverse_kinematics"
            ),
            Node(
                package="hexapod_controller",
                executable="body_pose_generator"
            ),
            Node(
                package="hexapod_controller",
                executable="tip_pose_generator"
            ),
            Node(
                package="hexapod_controller",
                executable="tip_trajectory_generator"
            )
        ]
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,  # Delay of 2 seconds
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager"
                ]
            )
        ]
    )

    simple_position_controller = TimerAction(
        period=4.0,  # Delay of 4 seconds
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_position_controller",
                    "--controller-manager",
                    "/controller_manager"
                ]
            )
        ]
    )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        simple_position_controller,
        group
    ])