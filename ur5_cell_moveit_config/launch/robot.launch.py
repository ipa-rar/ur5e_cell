from os.path import join
import pprint
from launch import LaunchDescription
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition, IfCondition
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("ur5_msa", package_name="ur5_cell_moveit_config").to_moveit_configs()
    #pprint.pprint(moveit_config.to_dict())
    ld = LaunchDescription()
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "use_rviz", 
            default_value=True)
        )
    

    virtual_joints_launch = (
        moveit_config.package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/rsp.launch.py")
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )
    # UR hardware specific things
    ld.add_action(
        Node(
            package="ur_robot_driver",
            executable="ur_ros2_control_node",
            parameters=[
                moveit_config.robot_description, 
                str(moveit_config.package_path / "config/ros2_controllers.yaml")],
            output="screen"
        )
    )    
    
    ld.add_action(
        Node(
            package="ur_robot_driver",
            condition=IfCondition("True") and UnlessCondition("False"),
            executable="dashboard_client",
            name="dashboard_client",
            output="screen",
            emulate_tty=True,
            parameters=[{"robot_ip": "192.168.56.2"}],
        )
    )
    
    ld.add_action(
        Node(
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
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )
    
    return ld