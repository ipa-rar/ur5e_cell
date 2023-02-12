from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ur_control_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])
        ),
        launch_arguments={
            "ur_type": "ur5e",              
            "use_fake_hardware": "true",
            "initial_joint_controller": "joint_trajectory_controller",
            "activate_joint_controller": "true",
            "robot_ip": "xxx.yyy.zzz.vvv",
            "description_package": "ur5e_cell_description",
            "description_file": "workcell.urdf.xacro",
            "launch_rviz": "true"
            }.items()
    )
    
    
    ld = LaunchDescription(
        [
            ur_control_launch
        ]
    )
    
    return ld