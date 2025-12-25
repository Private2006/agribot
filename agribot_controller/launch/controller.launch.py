import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )
    
    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True",
    )

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.15001", # Scale this by 2.143
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.47146", # Scale this by 2.143
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    
    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["agribot_diff_controller",
                "--controller-manager",
                "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller),
    )
    
    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["simple_velocity_controller",
                        "--controller-manager",
                        "/controller_manager"
                ]),
            
            Node(
                package="agribot_controller",
                executable="simple_controller.py",
                parameters=[{"wheel_radius": wheel_radius,
                            "wheel_separation": wheel_separation,
                            "use_sim_time": use_sim_time}])
        ]
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_simple_controller_arg,
            wheel_radius_arg,
            wheel_separation_arg,
            joint_state_broadcaster_spawner,
            diff_drive_controller,
            simple_controller,
        ]
    )