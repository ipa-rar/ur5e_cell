from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur5e_workcell", package_name="ur5e_cell_moveit_config")
        .robot_description(file_path="config/ur5e_workcell.urdf.xacro")
        .moveit_cpp(
            file_path=get_package_share_directory("ur5e_cell_pick_n_place")
            + "/config/moveitcpp.yaml"
        )
        .to_moveit_configs()
    )
    
    # MoveItCpp demo executable
    moveit_cpp_node = Node(
        name="pick_n_place_node",
        package="ur5e_cell_pick_n_place",
        executable="pick_n_place_node",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    
    return LaunchDescription(
        [
            moveit_cpp_node
        ]
    )