from launch import LaunchDescription
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros_extras.actions import LoadMoveitConfig, GenerateMoveitLaunch
from launch_ros.actions import Node

def generate_launch_description():
    load_moveit_config = LoadMoveitConfig(
        robot_name="ur5e_workcell_fake",
        package_name="ur5e_cell_moveit_config")
    
    generate_demo_ld = GenerateMoveitLaunch(
        function= generate_demo_launch)

    io_and_status_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["io_and_status_controller", "-c", "/controller_manager"],
    )
    
    ld = LaunchDescription(
        [
            load_moveit_config,
            generate_demo_ld,
            io_and_status_controller_spawner
        ]
    )
    
    return ld
