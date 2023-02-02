from os.path import join
import pprint
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
    generate_spawn_controllers_launch,
    generate_static_virtual_joint_tfs_launch,
    generate_warehouse_db_launch
)
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition, IfCondition
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros_extras.actions import LoadMoveitConfig, GenerateMoveitLaunch, LoadYaml
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    
    database_config = DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
            )


    debug_config = DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    
    rviz_config = DeclareBooleanLaunchArg(
            "use_rviz", 
            default_value=True,
            description="Whether to use RVIZ or not."    
        )
    
    robot_ip_config = DeclareLaunchArgument(
            name="robot_ip",
            default_value="192.168.56.2",
            description="The IP address of the robot to connect to."
        )
    
    load_servo_config = LoadYaml(
        package="ur5e_cell_moveit_config",
        file="config/moveit_servo.yaml",
        configuration_variable="moveit_servo"
    )
    
    load_moveit_config = LoadMoveitConfig(
        robot_name="ur5e_workcell_fake",
        package_name="ur5e_cell_moveit_config")

    generate_virtual_joint_ld = GenerateMoveitLaunch(
        function=generate_static_virtual_joint_tfs_launch
    )
    
    generate_rsp_ld = GenerateMoveitLaunch(
        function=generate_rsp_launch
    )
    
    generate_move_group_ld = GenerateMoveitLaunch(
        function=generate_move_group_launch
    )
    
    generate_moveit_rviz_ld = GenerateMoveitLaunch(
        function=generate_moveit_rviz_launch,
        condition=IfCondition(LaunchConfiguration("use_rviz"))
    )
    
    generate_warehouse_db_ld = GenerateMoveitLaunch(
        function=generate_warehouse_db_launch,
        condition=IfCondition(LaunchConfiguration("db"))
    )

    # UR hardware specific things
    ur_ros2_control_node = Node(
            package="ur_robot_driver",
            executable="ur_ros2_control_node",
            parameters=[
                {"robot_description": LaunchConfiguration("moveit_robot_description")}, 
                PathJoinSubstitution([LaunchConfiguration('moveit_package_path'), "config", "ros2_controllers.yaml"])
                ],
            output="screen"
        )
    
    ur_dashboard_client = Node(
            package="ur_robot_driver",
            condition=IfCondition("True") and UnlessCondition("False"),
            executable="dashboard_client",
            name="dashboard_client",
            output="screen",
            emulate_tty=True,
            parameters=[{"robot_ip": "192.168.56.2"}],
        )
    
    ur_controller_stopper_node = Node(
            package="ur_robot_driver",
            executable="controller_stopper_node",
            name="controller_stopper",
            output="screen",
            emulate_tty=True,
            condition=UnlessCondition("False"),
            parameters=[
                {"headless_mode": True},
                {"joint_controller_active": False},
                {
                    "consistent_controllers": [
                        "io_and_status_controller",
                        "force_torque_sensor_broadcaster",
                        "joint_state_broadcaster",
                        "speed_scaling_state_broadcaster",
                    ]
                },
            ],
        )
    
    # moveit_servo_node = Node(
    #     package="moveit_servo",
    #     executable="servo_node_main",
    #     parameters=[
    #         {"moveit_servo": LaunchConfiguration("moveit_servo")},
    #         {"robot_description": LaunchConfiguration("moveit_robot_description")},
    #         {"robot_description_semantic": LaunchConfiguration("moveit_robot_description_semantic")},
    #     ],
    #     output="screen",
    # )
    
    # container = ComposableNodeContainer(
    #     name="servo_joy_container",
    #     namespace="/",
    #     package="rclcpp_components",
    #     executable="component_container_mt",
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package="moveit_servo",
    #             plugin="moveit_servo::JoyToServoPub",
    #             name="controller_to_servo_node",
    #         ),
    #         ComposableNode(
    #             package="joy",
    #             plugin="joy::Joy",
    #             name="joy_node",
    #         ),
    #     ],
    #     output="screen",
    # )
    
    generate_controllers_ld = GenerateMoveitLaunch(
        function=generate_spawn_controllers_launch
    )
    
    ld = LaunchDescription(
        [
            database_config,
            debug_config,
            rviz_config,
            robot_ip_config,
            load_moveit_config,
            load_servo_config,
            generate_virtual_joint_ld,
            generate_rsp_ld,
            generate_move_group_ld,
            generate_moveit_rviz_ld,
            generate_warehouse_db_ld,
            ur_ros2_control_node,
            ur_dashboard_client,
            ur_controller_stopper_node,
            #container,
            #moveit_servo_node,
            generate_controllers_ld
        ]
    )
    return ld