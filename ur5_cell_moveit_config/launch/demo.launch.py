from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur5_msa_fake", package_name="ur5_cell_moveit_config").to_moveit_configs()
    ld = generate_demo_launch(moveit_config)
    return ld
