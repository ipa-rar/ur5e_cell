#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <geometry_msgs/msg/point_stamped.h>
#include <ur_msgs/srv/set_io.hpp>

bool open_gripper(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr set_io_client)
{
  ur_msgs::srv::SetIO::Request set_req1;
  set_req1.fun = (float)ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
  set_req1.pin = (float)ur_msgs::srv::SetIO::Request::PIN_DOUT0;
  set_req1.state = (float)ur_msgs::srv::SetIO::Request::STATE_OFF;
   RCLCPP_INFO(rclcpp::get_logger("set_io"), "%s", ur_msgs::srv::to_yaml(set_req1).c_str());
  auto fut = set_io_client->async_send_request(std::make_shared<ur_msgs::srv::SetIO::Request>(set_req1));
  auto fut_res = fut.wait_for(std::chrono::seconds(1));
  if(fut_res == std::future_status::timeout)
  {
    RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO failed with timeout.");
    return false;
  }

  if(!fut.get()->success)
  {
    RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO was not successful.");
    return false;
  }

  ur_msgs::srv::SetIO::Request reset_req2;
  reset_req2.fun = (float)ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
  reset_req2.pin = (float)ur_msgs::srv::SetIO::Request::PIN_DOUT1;
  reset_req2.state = (float)ur_msgs::srv::SetIO::Request::STATE_ON;

  fut = set_io_client->async_send_request(std::make_shared<ur_msgs::srv::SetIO::Request>(reset_req2));
  fut_res = fut.wait_for(std::chrono::seconds(1));
  if(fut_res == std::future_status::timeout)
  {
    RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO failed with timeout.");
    return false;
  }

  if(!fut.get()->success)
  {
    RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO was not successful.");
    return false;
  }
}

bool close_gripper(rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr set_io_client)
{
  ur_msgs::srv::SetIO::Request set_req2;
  set_req2.fun = (float)ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
  set_req2.pin = (float)ur_msgs::srv::SetIO::Request::PIN_DOUT1;
  set_req2.state = (float)ur_msgs::srv::SetIO::Request::STATE_OFF;
  
  RCLCPP_INFO(rclcpp::get_logger("set_io"), "%s", ur_msgs::srv::to_yaml(set_req2).c_str());
  auto fut = set_io_client->async_send_request(std::make_shared<ur_msgs::srv::SetIO::Request>(set_req2));
  auto fut_res = fut.wait_for(std::chrono::seconds(1));
  if(fut_res == std::future_status::timeout)
  {
    RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO failed with timeout.");
    return false;
  }

  if(!fut.get()->success)
  {
    RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO was not successful.");
    return false;
  }

  ur_msgs::srv::SetIO::Request reset_req1;
  reset_req1.fun = (float)ur_msgs::srv::SetIO::Request::FUN_SET_DIGITAL_OUT;
  reset_req1.pin = (float)ur_msgs::srv::SetIO::Request::PIN_DOUT0;
  reset_req1.state = (float)ur_msgs::srv::SetIO::Request::STATE_ON;

  fut = set_io_client->async_send_request(std::make_shared<ur_msgs::srv::SetIO::Request>(reset_req1));
  fut_res = fut.wait_for(std::chrono::seconds(1));
  if(fut_res == std::future_status::timeout)
  {
    RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO failed with timeout.");
    return false;
  }

  if(!fut.get()->success)
  {
    RCLCPP_INFO(rclcpp::get_logger("set_io"), "Set IO was not successful.");
    return false;
  }
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("run_moveit_cpp", "", node_options);
  std::string client_name = "/io_and_status_controller/set_io";
  auto set_io_client = node->create_client<ur_msgs::srv::SetIO>(client_name);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();
  rclcpp::sleep_for(std::chrono::seconds(1));
  if(!set_io_client->wait_for_service(std::chrono::seconds(3)))
  {
    RCLCPP_INFO(node->get_logger(), "No set_io service");
    rclcpp::shutdown();
    return 0;
  }
  static const std::string PLANNING_GROUP = "arm";

  auto moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(node);
  //rclcpp::sleep_for(std::chrono::seconds(1));
  moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();

  auto planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);

  // Set poses
  tf2::Quaternion q;
  q.setRPY(-3.14,0,1.57);

  geometry_msgs::msg::PoseStamped start_position;
  start_position.header.frame_id = "world";
  start_position.pose.position.x = -0.17;
  start_position.pose.position.y = 0.0;
  start_position.pose.position.z = 1.5;
  start_position.pose.orientation.x = q.getX();
  start_position.pose.orientation.y = q.getY();
  start_position.pose.orientation.z = q.getZ();
  start_position.pose.orientation.w = q.getW();


  geometry_msgs::msg::PoseStamped approach_object_1;
  approach_object_1.header.frame_id = "world";
  approach_object_1.pose.position.x = -0.17;
  approach_object_1.pose.position.y = 0.16;
  approach_object_1.pose.position.z = 1.2;
  approach_object_1.pose.orientation.x = q.getX();
  approach_object_1.pose.orientation.y = q.getY();
  approach_object_1.pose.orientation.z = q.getZ();
  approach_object_1.pose.orientation.w = q.getW();

  geometry_msgs::msg::PoseStamped pick_object_1;
  pick_object_1.header.frame_id = "world";
  pick_object_1.pose.position.x = -0.17;
  pick_object_1.pose.position.y = 0.16;
  pick_object_1.pose.position.z = 1.05;
  pick_object_1.pose.orientation.x = q.getX();
  pick_object_1.pose.orientation.y = q.getY();
  pick_object_1.pose.orientation.z = q.getZ();
  pick_object_1.pose.orientation.w = q.getW();

  geometry_msgs::msg::PoseStamped approach_object_2;
  approach_object_2.header.frame_id = "world";
  approach_object_2.pose.position.x = -0.17;
  approach_object_2.pose.position.y = -0.10;
  approach_object_2.pose.position.z = 1.2;
  approach_object_2.pose.orientation.x = q.getX();
  approach_object_2.pose.orientation.y = q.getY();
  approach_object_2.pose.orientation.z = q.getZ();
  approach_object_2.pose.orientation.w = q.getW();

  geometry_msgs::msg::PoseStamped pick_object_2;
  pick_object_2.header.frame_id = "world";
  pick_object_2.pose.position.x = -0.17;
  pick_object_2.pose.position.y = -0.10;
  pick_object_2.pose.position.z = 1.05;
  pick_object_2.pose.orientation.x = q.getX();
  pick_object_2.pose.orientation.y = q.getY();
  pick_object_2.pose.orientation.z = q.getZ();
  pick_object_2.pose.orientation.w = q.getW();

  planning_components->setStartStateToCurrentState();
  open_gripper(set_io_client);

  planning_components->setGoal(start_position, "tool_tip");
  const moveit_cpp::PlanningComponent::PlanSolution to_start_plan = planning_components->plan();
  if(!to_start_plan)
  {
    RCLCPP_INFO(node->get_logger(), "Planning to start failed.");
    rclcpp::shutdown();
    return 0;
  }
  moveit_cpp_ptr->execute("arm", to_start_plan.trajectory);

  planning_components->setStartStateToCurrentState();
  planning_components->setGoal(approach_object_1, "tool_tip");
  const moveit_cpp::PlanningComponent::PlanSolution to_approach1 = planning_components->plan();
  if(!to_approach1)
  {
    RCLCPP_INFO(node->get_logger(), "Planning to approach 1 failed.");
    rclcpp::shutdown();
    return 0;
  }
  moveit_cpp_ptr->execute("arm", to_approach1.trajectory);
  open_gripper(set_io_client);

  planning_components->setStartStateToCurrentState();
  planning_components->setGoal(pick_object_1, "tool_tip");
  const moveit_cpp::PlanningComponent::PlanSolution to_pick1 = planning_components->plan();
  if(!to_pick1)
  {
    RCLCPP_INFO(node->get_logger(), "Planning to pick 1 failed.");
    rclcpp::shutdown();
    return 0;
  }
  moveit_cpp_ptr->execute("arm", to_pick1.trajectory);

  close_gripper(set_io_client);

  planning_components->setStartStateToCurrentState();
  planning_components->setGoal(approach_object_1, "tool_tip");
  const moveit_cpp::PlanningComponent::PlanSolution to_approach1_1 = planning_components->plan();
  if(!to_approach1_1)
  {
    RCLCPP_INFO(node->get_logger(), "Planning to approach 1 failed.");
    rclcpp::shutdown();
    return 0;
  }
  moveit_cpp_ptr->execute("arm", to_approach1_1.trajectory);

  planning_components->setStartStateToCurrentState();
  planning_components->setGoal(approach_object_2, "tool_tip");
  const moveit_cpp::PlanningComponent::PlanSolution to_approach2 = planning_components->plan();
  if(!to_approach2)
  {
    RCLCPP_INFO(node->get_logger(), "Planning to aproach 2 failed.");
    rclcpp::shutdown();
    return 0;
  }
  moveit_cpp_ptr->execute("arm", to_approach2.trajectory);

  planning_components->setStartStateToCurrentState();
  planning_components->setGoal(pick_object_2, "tool_tip");
  const moveit_cpp::PlanningComponent::PlanSolution to_pick2 = planning_components->plan();
  if(!to_pick2)
  {
    RCLCPP_INFO(node->get_logger(), "Planning to pick 2 failed.");
    rclcpp::shutdown();
    return 0;
  }
  moveit_cpp_ptr->execute("arm", to_pick2.trajectory);

  open_gripper(set_io_client);

  planning_components->setStartStateToCurrentState();
  planning_components->setGoal(start_position, "tool_tip");
  const moveit_cpp::PlanningComponent::PlanSolution to_final_pos = planning_components->plan();
  if(!to_final_pos)
  {
    RCLCPP_INFO(node->get_logger(), "Planning to final position failed.");
    rclcpp::shutdown();
    return 0;
  }
  moveit_cpp_ptr->execute("arm", to_final_pos.trajectory);

  rclcpp::sleep_for(std::chrono::seconds(5));
  rclcpp::shutdown();
  return 0;
}
