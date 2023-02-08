#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("point_to_point_planner");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("point_to_point_planner", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();




  static const std::string PLANNING_GROUP = "arm";
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  geometry_msgs::msg::PoseStamped cpose = move_group.getCurrentPose();
  RCLCPP_INFO(LOGGER, geometry_msgs::msg::to_yaml(cpose).c_str());


  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(-3.14,0,1.57);
  geometry_msgs::msg::Pose target_pose1;
  geometry_msgs::msg::Pose target_pose2;
  geometry_msgs::msg::Pose target_pose3;
  geometry_msgs::msg::Pose target_pose4;

  std::vector<geometry_msgs::msg::Pose> waypoints;

  // Aproach area 1
  target_pose1.orientation.x = myQuaternion.getX();
  target_pose1.orientation.y = myQuaternion.getY();
  target_pose1.orientation.z = myQuaternion.getZ();
  target_pose1.orientation.w = myQuaternion.getW();

  //target_pose1.position.x = -0.171;
  target_pose1.position.x = -0.17;
  target_pose1.position.y = 0.16;
  target_pose1.position.z = 1.4;


  // Pick area 1
  target_pose2.orientation.x = myQuaternion.getX();
  target_pose2.orientation.y = myQuaternion.getY();
  target_pose2.orientation.z = myQuaternion.getZ();
  target_pose2.orientation.w = myQuaternion.getW();

  //target_pose1.position.x = -0.171;
  target_pose2.position.x = -0.17;
  target_pose2.position.y = 0.16;
  target_pose2.position.z = 1.25;


  // Aproach area 2
  target_pose3.orientation.x = myQuaternion.getX();
  target_pose3.orientation.y = myQuaternion.getY();
  target_pose3.orientation.z = myQuaternion.getZ();
  target_pose3.orientation.w = myQuaternion.getW();

  //target_pose1.position.x = -0.171;
  target_pose3.position.x = -0.17;
  target_pose3.position.y = -0.10;
  target_pose3.position.z = 1.4;



  // Pick area 2
  target_pose4.orientation.x = myQuaternion.getX();
  target_pose4.orientation.y = myQuaternion.getY();
  target_pose4.orientation.z = myQuaternion.getZ();
  target_pose4.orientation.w = myQuaternion.getW();

  //target_pose1.position.x = -0.171;
  target_pose4.position.x = -0.17;
  target_pose4.position.y = -0.10;
  target_pose4.position.z = 1.25;

  waypoints.push_back(target_pose1);
  waypoints.push_back(target_pose2);
  waypoints.push_back(target_pose1);
  waypoints.push_back(target_pose3);
  waypoints.push_back(target_pose4);
  waypoints.push_back(target_pose3);


  for (int i = 0; i < waypoints.size(); i++)
  {
    move_group.setPoseTarget(waypoints[i]);
    move_group.setMaxVelocityScalingFactor(1.3);
    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group.setPlannerId("LIN");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    if(!success) break;
    
    success = (move_group.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(LOGGER, "Executing plan 1(pose goal) %s", success ? "" : "FAILED");
    if(!success) break;

    //sleep(1.0);

  }
  




  rclcpp::shutdown();
  return 0;
}
