#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <franka_custom_interfaces/action/mtc_pick_object.hpp>

using MtcPickObject = franka_custom_interfaces::action::MtcPickObject;
using GoalHandleMtcPick = rclcpp_action::ServerGoalHandle<MtcPickObject>;
namespace mtc = moveit::task_constructor;

class MtcPickServer : public rclcpp::Node {
public:
    MtcPickServer() : Node("mtc_pick_server_cpp", rclcpp::NodeOptions().allow_undeclared_parameters(true)) {
        action_server_ = rclcpp_action::create_server<MtcPickObject>(
            this, "mtc_pick_object",
            std::bind(&MtcPickServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&MtcPickServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&MtcPickServer::handle_accepted, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "MTC Pick Action Server (C++ Nativo) OPERATIVO.");
    }

private:
    rclcpp_action::Server<MtcPickObject>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const MtcPickObject::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMtcPick>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMtcPick> goal_handle) {
        std::thread{std::bind(&MtcPickServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleMtcPick> goal_handle) {
        auto result = std::make_shared<MtcPickObject::Result>();
        auto goal = goal_handle->get_goal();
        std::string object_id = "target_cube";

        mtc::Task task;
        task.stages()->setName("fr3_pick_task");
        
        try {
            task.loadRobotModel(shared_from_this());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "RobotModel load failed: %s", e.what());
            result->success = false;
            goal_handle->abort(result);
            return;
        }

        // Setup Solver OMPL
        auto pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this(), "ompl");
        pipeline_planner->setPlannerId("RRTConnectkConfigDefault");
        this->set_parameter(rclcpp::Parameter("ompl.planning_plugin", "ompl_interface/OMPLPlanner"));

        // 1. Current State
        task.add(std::make_unique<mtc::stages::CurrentState>("current_state"));

        // 2. Spawn Object (Usa fr3_link0 come frame di riferimento radice)
        auto spawn_obj = std::make_unique<mtc::stages::ModifyPlanningScene>("spawn_object");
        moveit_msgs::msg::CollisionObject obj;
        obj.id = object_id;
        obj.header.frame_id = "fr3_link0"; 
        shape_msgs::msg::SolidPrimitive box;
        box.type = shape_msgs::msg::SolidPrimitive::BOX;
        box.dimensions = {0.04, 0.04, 0.04};
        obj.primitives.push_back(box);
        obj.primitive_poses.push_back(goal->target_pose.pose);
        obj.operation = moveit_msgs::msg::CollisionObject::ADD;
        spawn_obj->addObject(obj);
        task.add(std::move(spawn_obj));

        // 3. Open Hand
        auto open_hand = std::make_unique<mtc::stages::MoveTo>("open_hand", pipeline_planner);
        open_hand->setGroup("hand");
        open_hand->setGoal("open");
        task.add(std::move(open_hand));

        RCLCPP_INFO(this->get_logger(), "Calcolo IK in corso...");
        
        try {
            if (task.plan(3)) {
                RCLCPP_INFO(this->get_logger(), "Piano trovato. Invio esecuzione...");
                
                // CATTURA RISULTATO ESECUZIONE
                auto execution_result = task.execute(*task.solutions().front());
                
                if (execution_result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "ESECUZIONE COMPLETATA CON SUCCESSO.");
                    result->success = true;
                    goal_handle->succeed(result);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Errore durante l'esecuzione fisica: %d", execution_result.val);
                    result->success = false;
                    goal_handle->abort(result);
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Pianificazione fallita.");
                result->success = false;
                goal_handle->abort(result);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Eccezione MTC: %s", e.what());
            result->success = false;
            goal_handle->abort(result);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MtcPickServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}