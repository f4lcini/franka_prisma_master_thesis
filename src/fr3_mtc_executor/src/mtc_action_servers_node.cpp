#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>
#include <iostream>

// MTC Headers
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <franka_custom_interfaces/action/mtc_pick_object.hpp>
#include <franka_custom_interfaces/action/mtc_place_object.hpp>
#include <franka_custom_interfaces/action/mtc_move_home.hpp>

using namespace std::placeholders;
namespace mtc = moveit::task_constructor;

// ================= TASK FACTORY =================
class TaskFactory {
public:
    TaskFactory(const rclcpp::Node::SharedPtr& node,
                const mtc::solvers::PipelinePlannerPtr& pipeline,
                const mtc::solvers::CartesianPathPtr& cartesian)
        : node_(node), pipeline_planner_(pipeline), cartesian_planner_(cartesian) {}

    /**
     * @brief Creates a Pick task based ONLY on the target pose (no sampling object needed).
     * Bypasses the "object not in scene" error by using MoveTo instead of GenerateGraspPose.
     */
    mtc::Task createPickTask(const std::string& arm_group,
                            const std::string& hand_group,
                            const std::string& tcp_frame,
                            const geometry_msgs::msg::PoseStamped& target_pose,
                            double approach_dist,
                            const moveit::core::RobotModelConstPtr& model) {
        mtc::Task task;
        task.stages()->setName("Pick Pose - " + arm_group);
        task.setRobotModel(model);

        task.setProperty("group", arm_group);
        task.setProperty("eef", hand_group);
        task.setProperty("ik_frame", tcp_frame);

        // 1. Start from current state
        auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
        mtc::Stage* current_state_ptr = current_state.get();
        task.add(std::move(current_state));

        // 2. Open Hand
        auto open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", pipeline_planner_);
        open_hand->setGroup(hand_group);
        open_hand->setGoal("open");
        task.add(std::move(open_hand));

        // 3. Connect to Pre-grasp
        auto connect = std::make_unique<mtc::stages::Connect>(
            "connect to pre-approach", mtc::stages::Connect::GroupPlannerVector{{arm_group, pipeline_planner_}});
        connect->setTimeout(5.0);
        task.add(std::move(connect));

        // 4. Move to Grasp Pose (approach_dist above target)
        auto move_to_grasp = std::make_unique<mtc::stages::MoveTo>("move to grasp pose", pipeline_planner_);
        move_to_grasp->setGroup(arm_group);
        move_to_grasp->setIKFrame(tcp_frame);
        move_to_grasp->setGoal(target_pose);
        task.add(std::move(move_to_grasp));

        // 5. Close Hand
        auto close_hand = std::make_unique<mtc::stages::MoveTo>("close hand", pipeline_planner_);
        close_hand->setGroup(hand_group);
        close_hand->setGoal("close");
        task.add(std::move(close_hand));

        // 6. Lift
        auto lift = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian_planner_);
        lift->setGroup(arm_group);
        lift->setIKFrame(tcp_frame);
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "world";
        vec.vector.z = approach_dist;
        lift->setDirection(vec);
        task.add(std::move(lift));

        return task;
    }

    /**
     * @brief Creates a Place task based ONLY on requested pose.
     */
    mtc::Task createPlaceTask(const std::string& arm_group,
                             const std::string& hand_group,
                             const std::string& tcp_frame,
                             const geometry_msgs::msg::PoseStamped& place_pose,
                             double retreat_dist,
                             const moveit::core::RobotModelConstPtr& model) {
        mtc::Task task;
        task.stages()->setName("Place Pose - " + arm_group);
        task.setRobotModel(model);
        task.setProperty("group", arm_group);
        task.setProperty("eef", hand_group);
        task.setProperty("ik_frame", tcp_frame);

        task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

        auto connect = std::make_unique<mtc::stages::Connect>(
            "connect", mtc::stages::Connect::GroupPlannerVector{{arm_group, pipeline_planner_}});
        connect->setTimeout(5.0);
        task.add(std::move(connect));

        auto move_to_place = std::make_unique<mtc::stages::MoveTo>("move to place pose", pipeline_planner_);
        move_to_place->setGroup(arm_group);
        move_to_place->setIKFrame(tcp_frame);
        move_to_place->setGoal(place_pose);
        task.add(std::move(move_to_place));

        auto open_hand = std::make_unique<mtc::stages::MoveTo>("open hand", pipeline_planner_);
        open_hand->setGroup(hand_group);
        open_hand->setGoal("open");
        task.add(std::move(open_hand));

        auto retreat = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner_);
        retreat->setGroup(arm_group);
        retreat->setIKFrame(tcp_frame);
        geometry_msgs::msg::Vector3Stamped vec;
        vec.header.frame_id = "world";
        vec.vector.z = retreat_dist;
        retreat->setDirection(vec);
        task.add(std::move(retreat));

        return task;
    }

    mtc::Task createHomeTask(const std::string& arm_group, const moveit::core::RobotModelConstPtr& model) {
        mtc::Task task;
        task.stages()->setName("Home - " + arm_group);
        task.setRobotModel(model);
        task.setProperty("group", arm_group);

        task.add(std::make_unique<mtc::stages::CurrentState>("current state"));

        auto move_home = std::make_unique<mtc::stages::MoveTo>("move home", pipeline_planner_);
        move_home->setGroup(arm_group);
        move_home->setGoal("ready"); 
        task.add(std::move(move_home));
        return task;
    }

private:
    rclcpp::Node::SharedPtr node_;
    mtc::solvers::PipelinePlannerPtr pipeline_planner_;
    mtc::solvers::CartesianPathPtr cartesian_planner_;
};

// ================= SERVER =================
class MtcMonolithicServer : public rclcpp::Node {
public:
    using PickAction = franka_custom_interfaces::action::MtcPickObject;
    using PlaceAction = franka_custom_interfaces::action::MtcPlaceObject;
    using HomeAction = franka_custom_interfaces::action::MtcMoveHome;

    MtcMonolithicServer() : Node("mtc_monolithic_server") {}

    void initialize() {
        RCLCPP_INFO(this->get_logger(), "Initializing Bimanual MTC Server...");
        auto node = this->shared_from_this();

        mgi_left_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "franka1_arm");
        mgi_right_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "franka2_arm");

        pipeline_planner_ = std::make_shared<mtc::solvers::PipelinePlanner>(node);
        cartesian_planner_ = std::make_shared<mtc::solvers::CartesianPath>();

        factory_ = std::make_unique<TaskFactory>(node, pipeline_planner_, cartesian_planner_);

        pick_server_ = rclcpp_action::create_server<PickAction>(this, "mtc_pick_object",
            std::bind(&MtcMonolithicServer::handle_pick_goal, this, _1, _2),
            std::bind(&MtcMonolithicServer::handle_pick_cancel, this, _1),
            std::bind(&MtcMonolithicServer::handle_pick_accepted, this, _1));

        place_server_ = rclcpp_action::create_server<PlaceAction>(this, "mtc_place_object",
            std::bind(&MtcMonolithicServer::handle_place_goal, this, _1, _2),
            std::bind(&MtcMonolithicServer::handle_place_cancel, this, _1),
            std::bind(&MtcMonolithicServer::handle_place_accepted, this, _1));

        home_server_ = rclcpp_action::create_server<HomeAction>(this, "mtc_move_home",
            std::bind(&MtcMonolithicServer::handle_home_goal, this, _1, _2),
            std::bind(&MtcMonolithicServer::handle_home_cancel, this, _1),
            std::bind(&MtcMonolithicServer::handle_home_accepted, this, _1));

        RCLCPP_INFO(this->get_logger(), "Ready for bimanual tasks.");
    }

private:
    std::string get_arm_group(const std::string& arm) { return (arm == "left_arm") ? "franka1_arm" : "franka2_arm"; }
    std::string get_hand_group(const std::string& arm) { return (arm == "left_arm") ? "franka1_hand" : "franka2_hand"; }
    std::string get_tcp_frame(const std::string& arm) { return (arm == "left_arm") ? "franka1_fr3_hand_tcp" : "franka2_fr3_hand_tcp"; }

    rclcpp_action::GoalResponse handle_pick_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const PickAction::Goal>) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
    rclcpp_action::GoalResponse handle_place_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const PlaceAction::Goal>) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }
    rclcpp_action::GoalResponse handle_home_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const HomeAction::Goal>) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; }

    rclcpp_action::CancelResponse handle_pick_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickAction>>) { return rclcpp_action::CancelResponse::ACCEPT; }
    rclcpp_action::CancelResponse handle_place_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PlaceAction>>) { return rclcpp_action::CancelResponse::ACCEPT; }
    rclcpp_action::CancelResponse handle_home_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<HomeAction>>) { return rclcpp_action::CancelResponse::ACCEPT; }

    void handle_pick_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickAction>> gh) { std::thread{std::bind(&MtcMonolithicServer::execute_pick, this, _1), gh}.detach(); }
    void handle_place_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PlaceAction>> gh) { std::thread{std::bind(&MtcMonolithicServer::execute_place, this, _1), gh}.detach(); }
    void handle_home_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<HomeAction>> gh) { std::thread{std::bind(&MtcMonolithicServer::execute_home, this, _1), gh}.detach(); }

    void execute_pick(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickAction>> goal_handle) {
        auto result = std::make_shared<PickAction::Result>();
        const auto goal = goal_handle->get_goal();
        auto mgi = (goal->arm == "left_arm") ? mgi_left_ : mgi_right_;
        RCLCPP_INFO(this->get_logger(), "Executing Pick Pose for %s", goal->arm.c_str());
        auto task = factory_->createPickTask(get_arm_group(goal->arm), get_hand_group(goal->arm), get_tcp_frame(goal->arm), goal->target_pose, goal->approach_distance, mgi->getRobotModel());
        try {
            task.init();
            if (task.plan(5)) {
                task.execute(*task.solutions().front());
                result->success = true; goal_handle->succeed(result); return;
            }
        } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what()); }
        goal_handle->abort(result);
    }

    void execute_place(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PlaceAction>> goal_handle) {
        auto result = std::make_shared<PlaceAction::Result>();
        const auto goal = goal_handle->get_goal();
        auto mgi = (goal->arm == "left_arm") ? mgi_left_ : mgi_right_;
        RCLCPP_INFO(this->get_logger(), "Executing Place Pose for %s", goal->arm.c_str());
        auto task = factory_->createPlaceTask(get_arm_group(goal->arm), get_hand_group(goal->arm), get_tcp_frame(goal->arm), goal->place_pose, goal->retreat_distance, mgi->getRobotModel());
        try {
            task.init();
            if (task.plan(5)) {
                task.execute(*task.solutions().front());
                result->success = true; goal_handle->succeed(result); return;
            }
        } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what()); }
        goal_handle->abort(result);
    }

    void execute_home(const std::shared_ptr<rclcpp_action::ServerGoalHandle<HomeAction>> goal_handle) {
        auto result = std::make_shared<HomeAction::Result>();
        auto mgi = (goal_handle->get_goal()->arm == "left_arm") ? mgi_left_ : mgi_right_;
        RCLCPP_INFO(this->get_logger(), "Executing Home for %s", goal_handle->get_goal()->arm.c_str());
        auto task = factory_->createHomeTask(get_arm_group(goal_handle->get_goal()->arm), mgi->getRobotModel());
        try {
            task.init();
            if (task.plan(1)) {
                task.execute(*task.solutions().front());
                result->success = true; goal_handle->succeed(result); return;
            }
        } catch (const std::exception& e) { RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what()); }
        goal_handle->abort(result);
    }

    std::unique_ptr<TaskFactory> factory_;
    std::shared_ptr<mtc::solvers::PipelinePlanner> pipeline_planner_;
    std::shared_ptr<mtc::solvers::CartesianPath> cartesian_planner_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi_left_, mgi_right_;
    rclcpp_action::Server<PickAction>::SharedPtr pick_server_;
    rclcpp_action::Server<PlaceAction>::SharedPtr place_server_;
    rclcpp_action::Server<HomeAction>::SharedPtr home_server_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MtcMonolithicServer>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}