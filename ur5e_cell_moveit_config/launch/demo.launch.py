from launch import LaunchDescription
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros_extras.actions import LoadMoveitConfig, GenerateMoveitLaunch

def generate_launch_description():
    load_moveit_config = LoadMoveitConfig(
        robot_name="ur5e_workcell_fake",
        package_name="ur5e_cell_moveit_config")
    
    generate_demo_ld = GenerateMoveitLaunch(
        function= generate_demo_launch)
    
    ld = LaunchDescription(
        [
            load_moveit_config,
            generate_demo_ld
        ]
    )
    
    return ld
