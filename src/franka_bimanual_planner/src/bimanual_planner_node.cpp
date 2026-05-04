#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "franka_custom_interfaces/action/parallel_move.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/position_constraint.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "moveit_msgs/srv/get_motion_plan.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace franka_bimanual_planner
{

class BimanualPlannerNode : public rclcpp::Node
{
public:
  using ParallelMove = franka_custom_interfaces::action::ParallelMove;
  using GoalHandleParallelMove = rclcpp_action::ServerGoalHandle<ParallelMove>;

  BimanualPlannerNode(const rclcpp::NodeOptions & options, rclcpp::Node::SharedPtr mg_node)
  : Node("bimanual_planner", options), move_group_node_(mg_node)
  {
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    move_group_right_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "franka1_manipulator");
    move_group_left_  = std::make_shared<moveit::planning_interface::MoveGroupInterface>(move_group_node_, "franka2_manipulator");

    planning_client_ = this->create_client<moveit_msgs::srv::GetMotionPlan>("plan_kinematic_path");

    move_group_right_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_left_->setPlanningPipelineId("pilz_industrial_motion_planner");

    // Direct FJT clients to bypass MoveIt execution bottleneck
    fjt_client_right_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
                         this, "franka1_arm_controller/follow_joint_trajectory", callback_group_);
    fjt_client_left_  = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
                         this, "franka2_arm_controller/follow_joint_trajectory", callback_group_);

    this->action_server_ = rclcpp_action::create_server<ParallelMove>(
      this, "parallel_move",
      std::bind(&BimanualPlannerNode::handle_goal, this, _1, _2),
      std::bind(&BimanualPlannerNode::handle_cancel, this, _1),
      std::bind(&BimanualPlannerNode::handle_accepted, this, _1),
      rcl_action_server_get_default_options(),
      callback_group_
    );

    RCLCPP_INFO(this->get_logger(), "🚀 Bimanual Planner Server (Ready with Direct Controller Access)");
  }

private:
  rclcpp_action::Server<ParallelMove>::SharedPtr action_server_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Node::SharedPtr move_group_node_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_right_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_left_;
  rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::SharedPtr planning_client_;

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr fjt_client_right_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr fjt_client_left_;

  std::mutex mutex_right_;
  std::mutex mutex_left_;

  void apply_constraints(moveit::planning_interface::MoveGroupInterface & move_group,
                         const std::string & arm, const std::string & tcp_frame)
  {
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::PositionConstraint pc;
    pc.header.frame_id = "world";
    pc.link_name = tcp_frame;

    shape_msgs::msg::SolidPrimitive sp;
    sp.type = sp.BOX;
    sp.dimensions = {2.2, 2.0, 2.2};

    geometry_msgs::msg::Pose center;
    center.position.y = 0.5;
    center.position.z = 0.5;
    center.orientation.w = 1.0;

    if (arm == "right") { center.position.x = 1.25; }
    else { center.position.x = -0.05; }

    pc.constraint_region.primitives.push_back(sp);
    pc.constraint_region.primitive_poses.push_back(center);
    pc.weight = 1.0;

    constraints.position_constraints.push_back(pc);
    move_group.setPathConstraints(constraints);
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ParallelMove::Goal> goal)
  {
    (void)uuid; (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleParallelMove> goal_handle)
  {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleParallelMove> goal_handle)
  {
    std::thread{ [this, goal_handle]() { this->execute(goal_handle); } }.detach();
  }

  void execute(const std::shared_ptr<GoalHandleParallelMove> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ParallelMove::Result>();

    bool is_right = (goal->arm == "right");
    std::string arm_group = is_right ? "franka1_manipulator" : "franka2_manipulator";

    auto & move_group_ptr = is_right ? move_group_right_ : move_group_left_;
    auto & fjt_client    = is_right ? fjt_client_right_ : fjt_client_left_;
    auto & arm_mutex     = is_right ? mutex_right_ : mutex_left_;

    std::lock_guard<std::mutex> lock(arm_mutex);

    try {
      move_group_ptr->setPlanningTime(10.0);
      move_group_ptr->setStartStateToCurrentState();

      if (!goal->joint_target.empty()) {
        move_group_ptr->setJointValueTarget(goal->joint_target);
      } else {
        auto target_pose_msg = goal->target_pose;
        if (target_pose_msg.header.frame_id.empty()) {
          target_pose_msg.header.frame_id = "world";
        }
        move_group_ptr->setPoseTarget(target_pose_msg);
      }

      if (!goal->is_handover) {
        apply_constraints(*move_group_ptr, goal->arm,
          is_right ? "franka1_fr3_hand_tcp" : "franka2_fr3_hand_tcp");
      } else {
        move_group_ptr->clearPathConstraints();
      }

      // Pipeline selection
      std::string active_planner = goal->planner_id;
      if (active_planner.empty()) active_planner = "PTP";

      if (active_planner == "ompl" || active_planner == "RRTConnect") {
        move_group_ptr->setPlanningPipelineId("ompl");
        if (active_planner == "ompl") active_planner = "RRTConnectkConfigDefault";
      } else {
        move_group_ptr->setPlanningPipelineId("pilz_industrial_motion_planner");
        if (active_planner == "PTP" || active_planner.empty()) active_planner = "PTP";
      }

      RCLCPP_INFO(this->get_logger(), "🚀 [%s] Pipeline: %s | Planner: %s",
                  arm_group.c_str(),
                  move_group_ptr->getPlanningPipelineId().c_str(),
                  active_planner.c_str());

      move_group_ptr->setPlannerId(active_planner);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto plan_result = move_group_ptr->plan(plan);

      if (plan_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        control_msgs::action::FollowJointTrajectory::Goal fjt_goal;
        fjt_goal.trajectory = plan.trajectory_.joint_trajectory;

        RCLCPP_INFO(this->get_logger(), "📦 [%s] Sending trajectory (%zu points) to JTC",
                    arm_group.c_str(), fjt_goal.trajectory.points.size());
        for (const auto & name : fjt_goal.trajectory.joint_names) {
          RCLCPP_INFO(this->get_logger(), "🔗 Joint: %s", name.c_str());
        }

        // --- HOLD PHASE: prevents libfranka acceleration discontinuity at startup ---
        // Insert a 0.5s "hold" at the start where the robot is commanded to stay still.
        // This gives libfranka time to initialize its motion generator before moving.
        {
          auto current_state = move_group_ptr->getCurrentState();
          std::vector<double> current_joints;
          current_state->copyJointGroupPositions(move_group_ptr->getName(), current_joints);
          size_t n_joints = current_joints.size();

          const double hold_duration = 0.5;

          // Shift all existing points forward by hold_duration
          for (auto & pt : fjt_goal.trajectory.points) {
            auto old_t = rclcpp::Duration(pt.time_from_start).seconds();
            pt.time_from_start = rclcpp::Duration::from_seconds(old_t + hold_duration);
          }

          // Insert hold point at t=0: current pos, zero vel/acc
          trajectory_msgs::msg::JointTrajectoryPoint hold_point;
          hold_point.positions = current_joints;
          hold_point.velocities.assign(n_joints, 0.0);
          hold_point.accelerations.assign(n_joints, 0.0);
          hold_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
          fjt_goal.trajectory.points.insert(fjt_goal.trajectory.points.begin(), hold_point);

          // Second point: same position, zero vel/acc (smooth takeoff)
          if (fjt_goal.trajectory.points.size() > 1) {
            fjt_goal.trajectory.points[1].velocities.assign(n_joints, 0.0);
            fjt_goal.trajectory.points[1].accelerations.assign(n_joints, 0.0);
            fjt_goal.trajectory.points[1].positions = current_joints;
          }

          fjt_goal.trajectory.header.stamp = this->now();
        }

        if (!fjt_client->wait_for_action_server(5s)) {
          RCLCPP_ERROR(this->get_logger(), "❌ [%s] JTC Action Server not available!", arm_group.c_str());
          goal_handle->abort(result);
          return;
        }

        auto goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
        auto future_goal_handle = fjt_client->async_send_goal(fjt_goal, goal_options);

        auto goal_handle_fjt = future_goal_handle.get();
        if (!goal_handle_fjt) {
          RCLCPP_ERROR(this->get_logger(), "❌ [%s] Goal rejected by JTC", arm_group.c_str());
          goal_handle->abort(result);
          return;
        }

        auto future_result = fjt_client->async_get_result(goal_handle_fjt);
        auto fjt_result = future_result.get();

        if (fjt_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          result->success = true;
          result->message = "Success";
          RCLCPP_INFO(this->get_logger(), "✅ [%s] Execution completed.", arm_group.c_str());
          if (goal_handle->is_active()) goal_handle->succeed(result);
        } else {
          result->success = false;
          result->message = "Execution failed";
          RCLCPP_ERROR(this->get_logger(), "❌ [%s] Execution failed.", arm_group.c_str());
          if (goal_handle->is_active()) goal_handle->abort(result);
        }
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "⏳ [%s] Planning failed or no path found.", arm_group.c_str());
        result->success = false;
        result->message = "Planning failed";
        goal_handle->abort(result);
      }
    } catch (const std::exception & e) {
      result->success = false;
      result->message = e.what();
      if (goal_handle->is_active()) goal_handle->abort(result);
    }
  }
};

} // namespace franka_bimanual_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  {
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto move_group_node = rclcpp::Node::make_shared("bimanual_mg_interface", options);
    auto main_node = std::make_shared<franka_bimanual_planner::BimanualPlannerNode>(options, move_group_node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(main_node);
    executor.add_node(move_group_node);

    RCLCPP_INFO(main_node->get_logger(), "🌐 Spinning on MultiThreadedExecutor...");
    executor.spin();
  }

  rclcpp::shutdown();
  return 0;
}
