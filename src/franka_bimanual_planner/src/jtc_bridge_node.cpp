// jtc_bridge_node.cpp
//
// "Fake JTC Bridge": Pretends to be a FollowJointTrajectory action server
// for both franka1 and franka2 arm controllers.
// When MoveIt (or RViz) sends a trajectory, this node converts each point
// via Forward Kinematics and publishes cartesian setpoints to the
// cartesian_impedance_left/right controllers (the IDRA official approach).
//
// This enables RViz "Plan & Execute" to work without the JointTrajectoryController.

#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_state/robot_state.h"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Geometry>

using namespace std::chrono_literals;
using FJT = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FJT>;

class JTCBridgeNode : public rclcpp::Node
{
public:
  JTCBridgeNode(const rclcpp::NodeOptions & options)
  : Node("jtc_bridge", options)
  {
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // Publishers to the cartesian impedance controllers
    pub_right_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/cartesian_impedance_right/commands", 10);
    pub_left_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/cartesian_impedance_left/commands", 10);

    // Fake JTC action servers — MoveIt / RViz will connect to these
    server_right_ = rclcpp_action::create_server<FJT>(
      this, "franka1_arm_controller/follow_joint_trajectory",
      [this](auto uuid, auto goal) { (void)uuid; (void)goal; return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](auto gh) { std::thread([this, gh]{ execute(gh, true); }).detach(); },
      rcl_action_server_get_default_options(), cb_group_);

    server_left_ = rclcpp_action::create_server<FJT>(
      this, "franka2_arm_controller/follow_joint_trajectory",
      [this](auto uuid, auto goal) { (void)uuid; (void)goal; return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](auto gh) { std::thread([this, gh]{ execute(gh, false); }).detach(); },
      rcl_action_server_get_default_options(), cb_group_);

    RCLCPP_INFO(this->get_logger(), "🌉 JTC Bridge ready — franka1 & franka2 fake action servers active");
  }

  // Called after construction to load the robot model (needs robot_description param)
  void init_robot_model()
  {
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      shared_from_this(), "robot_description");
    robot_model_ = robot_model_loader_->getModel();

    if (!robot_model_) {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to load robot model! Check robot_description param.");
    } else {
      RCLCPP_INFO(this->get_logger(), "✅ Robot model loaded: %s", robot_model_->getName().c_str());
    }
  }

private:
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  rclcpp_action::Server<FJT>::SharedPtr server_right_;
  rclcpp_action::Server<FJT>::SharedPtr server_left_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_right_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_left_;

  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;

  void execute(std::shared_ptr<GoalHandleFJT> goal_handle, bool is_right)
  {
    const std::string arm_name = is_right ? "franka1" : "franka2";
    const std::string arm_group = is_right ? "franka1_manipulator" : "franka2_manipulator";
    const std::string tcp_frame = is_right ? "franka1_fr3_hand_tcp" : "franka2_fr3_hand_tcp";
    auto & pub = is_right ? pub_right_ : pub_left_;

    RCLCPP_INFO(this->get_logger(), "🌉 [%s] Bridge received trajectory", arm_name.c_str());

    if (!robot_model_) {
      RCLCPP_ERROR(this->get_logger(), "❌ Robot model not loaded, cannot execute.");
      goal_handle->abort(std::make_shared<FJT::Result>());
      return;
    }

    auto robot_state = std::make_shared<moveit::core::RobotState>(robot_model_);
    const moveit::core::JointModelGroup * jmg = robot_model_->getJointModelGroup(arm_group);

    if (!jmg) {
      RCLCPP_ERROR(this->get_logger(), "❌ JMG '%s' not found", arm_group.c_str());
      goal_handle->abort(std::make_shared<FJT::Result>());
      return;
    }

    const auto & points = goal_handle->get_goal()->trajectory.points;
    RCLCPP_INFO(this->get_logger(), "📡 [%s] Streaming %zu points as cartesian setpoints...",
                arm_name.c_str(), points.size());

    for (size_t i = 0; i < points.size(); ++i) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(std::make_shared<FJT::Result>());
        return;
      }

      // FK: joint positions → cartesian pose
      robot_state->setJointGroupPositions(jmg, points[i].positions);
      robot_state->update();

      const Eigen::Isometry3d & ee = robot_state->getGlobalLinkTransform(tcp_frame);
      Eigen::Vector3d pos = ee.translation();
      Eigen::Quaterniond quat(ee.rotation());

      // Publish [x, y, z, w, qx, qy, qz]
      std_msgs::msg::Float64MultiArray msg;
      msg.data = {pos.x(), pos.y(), pos.z(),
                  quat.w(), quat.x(), quat.y(), quat.z()};
      pub->publish(msg);

      // Respect trajectory timing
      if (i + 1 < points.size()) {
        double t_curr = rclcpp::Duration(points[i].time_from_start).seconds();
        double t_next = rclcpp::Duration(points[i + 1].time_from_start).seconds();
        double dt = std::max(0.01, std::min(0.2, t_next - t_curr));
        rclcpp::sleep_for(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(dt)));
      } else {
        rclcpp::sleep_for(500ms);  // Hold final position
      }
    }

    auto result = std::make_shared<FJT::Result>();
    result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "✅ [%s] Bridge execution completed.", arm_name.c_str());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<JTCBridgeNode>(options);

  // Load robot model AFTER the node is shared_ptr (needed for shared_from_this)
  node->init_robot_model();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
