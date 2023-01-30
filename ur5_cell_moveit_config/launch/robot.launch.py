from os.path import join
import pprint
from typing import Callable, List, Optional
from launch import LaunchDescription
from launch import LaunchContext
from launch import LaunchDescriptionEntity
from launch.action import Action
from launch.actions import SetLaunchConfiguration
from launch.actions import OpaqueFunction
from launch import SomeSubstitutionsType
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_static_virtual_joint_tfs_launch,
    generate_rsp_launch,
    generate_move_group_launch,
    generate_warehouse_db_launch,
    generate_spawn_controllers_launch,
    generate_moveit_rviz_launch
    )   
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition, IfCondition
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

class MoveItConfigLoader(Action):
    def __init__(
        self,
        robot_name: SomeSubstitutionsType,
        package_name: SomeSubstitutionsType,
        mappings: Optional[dict] = None,
        **kwargs
    ) -> None:
        super().__init__(**kwargs)
        self.__robot_name = normalize_to_list_of_substitutions(robot_name)
        self.__package_name = normalize_to_list_of_substitutions(package_name)
        self.__mappings = mappings
        
    def execute(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        moveit_config_builder = MoveItConfigsBuilder(
            robot_name=perform_substitutions(context, self.__robot_name), 
            package_name=perform_substitutions(context, self.__package_name)
            )
        moveit_config_builder.robot_description(mappings=self.__mappings)
        moveit_config = moveit_config_builder.to_moveit_configs()
        
        # Add the moveit config to the launch context
        context.extend_locals({'moveit_config': moveit_config})
        
        #Create important launch configuration variables
        moveit_package_path = SetLaunchConfiguration(
            "moveit_package_path", 
            value=str(moveit_config.package_path))
        moveit_robot_description = SetLaunchConfiguration(
            "moveit_robot_description", 
            value=moveit_config.robot_description)
        moveit_ros2_controllers_file = SetLaunchConfiguration("moveit_ros2_controllers_file", value=str(moveit_config.package_path / "config/ros2_controllers.yaml"))
        return [moveit_package_path, moveit_robot_description, moveit_ros2_controllers_file]
        
class IncludeMoveitLaunchDescription(Action):
    def __init__(
        self, *,
        function: Callable,
        **kwargs) -> None:
        super().__init__(**kwargs)
        self.__function = function
    
    def execute(self, context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:
        return [self.__function(context.get_locals_as_dict()["moveit_config"])]


def generate_launch_description():
    ld = LaunchDescription()
    
    ld.add_action(DeclareLaunchArgument(
        "robot_name",
        default_value="ur5_msa",
        description="The name of the robot - should be prefix of the xacro describing the robot (robot_name.urdf.xacro)."
    ))

    ld.add_action(DeclareLaunchArgument(
        "package_name",
        default_value="ur5_msa",
        description="The name of the moveit config package."
    ))
    
    ld.add_action(DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.56.2",
        description="IP address of the robot."
    ))
    
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
    
    ld.add_action(
        MoveItConfigLoader(
            "ur5_msa", 
            "ur5_cell_moveit_config", 
            mappings={
                "use_fake_hardware":"True",
                "fake_sensor_commands":"True",
                }
            )
    )
    
    

    ld.add_action(
        IncludeMoveitLaunchDescription(
            function=generate_static_virtual_joint_tfs_launch)
    )

    ld.add_action(
        IncludeMoveitLaunchDescription(
            function=generate_rsp_launch)
    )
    
    ld.add_action(
        IncludeMoveitLaunchDescription(
            function=generate_moveit_rviz_launch)
    )

    ld.add_action(
        IncludeMoveitLaunchDescription(
            function=generate_move_group_launch,
        condition=IfCondition(LaunchConfiguration("use_rviz")))
    )
    
    ld.add_action(
        IncludeMoveitLaunchDescription(
            function=generate_warehouse_db_launch,
        condition=IfCondition(LaunchConfiguration("db")))
    )

    # UR hardware specific things    
    ld.add_action(
        Node(
            package="ur_robot_driver",
            executable="ur_ros2_control_node",
            parameters=[
                LaunchConfiguration('moveit_robot_description'), 
                LaunchConfiguration('moveit_ros2_controllers_file')],
            output="screen"
        )
    )    
    
    ld.add_action(
        Node(
            package="ur_robot_driver",
            executable="dashboard_client",
            name="dashboard_client",
            output="screen",
            emulate_tty=True,
            parameters=[{"robot_ip": LaunchConfiguration("robot_ip")}],
        )
    )
    
    ld.add_action(
        Node(
            package="ur_robot_driver",
            executable="controller_stopper_node",
            name="controller_stopper",
            output="screen",
            emulate_tty=True,
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
        IncludeMoveitLaunchDescription(
            function=generate_spawn_controllers_launch)
    )
    
    return ld