#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <std_msgs/msg/float64.hpp>
#include "double_pendulum_interfaces/msg/joint_target.hpp"
#include "double_pendulum_interfaces/msg/pose_target.hpp"
#include "double_pendulum_interfaces/msg/named_target.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using String_Msg = double_pendulum_interfaces::msg::NamedTarget;
using Joint_Arr = double_pendulum_interfaces::msg::JointTarget;
using Pose_Msg = double_pendulum_interfaces::msg::PoseTarget;

using namespace std::placeholders;

using StartState = moveit_msgs::msg::RobotState;
using RobotTraj = moveit_msgs::msg::RobotTrajectory;
using PlanTime = std_msgs::msg::Float64;

class Commander
{
public:
    Commander(std::shared_ptr<rclcpp::Node> node)
    {
        node_ = node;
        planned_ = false;
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);

        named_target_sub_ = node_->create_subscription<String_Msg>("named_target", 10, std::bind(&Commander::NamedTargetCallback, this, _1));
        joint_target_sub_ = node_->create_subscription<Joint_Arr>("joint_target", 10, std::bind(&Commander::JointTargetCallback, this, _1));
        pose_target_sub_ = node_->create_subscription<Pose_Msg>("pose_target", 10, std::bind(&Commander::PoseTargetCallback, this, _1));

        robot_traj_pub_ = node->create_publisher<RobotTraj>("/robot_traj_getter", 10);
        robot_start_state_pub_ = node->create_publisher<StartState>("/robot_plan_start_state", 10);
        robot_plan_time_pub_ = node->create_publisher<PlanTime>("/robot_plan_time", 10);

        timer_ = node->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&Commander::PublisherCallback, this)
        );

    }

    void goToNamedTarget(const std::string &name) 
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);
        planPath(arm_);
    }

    void goToJointTarget(const std::vector<double> &joints) 
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);
        planPath(arm_);
    }

    void goToPoseTarget(double x, double y, double z, 
                        double roll, double pitch, double yaw, bool cartesian_path=false) 
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q = q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "base_link";
        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;
        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();

        if (!cartesian_path) {
            arm_->setPoseTarget(target_pose);
            planPath(arm_);
        } else {
            // Not implemented
            return;
        }
        
    }


private:

    void planPath(const std::shared_ptr<MoveGroupInterface> &interface) 
    {
        MoveGroupInterface::Plan plan;
        bool success = (interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            planned_ = true;
            most_recent_plan_ = plan;
        }
    }

    void NamedTargetCallback(const String_Msg::SharedPtr msg) {
        goToNamedTarget(msg->name);
    }

    void JointTargetCallback(const Joint_Arr::SharedPtr arr) {
        // Expecting array of joints
        std::vector<double> q = arr->joint_data;
        goToJointTarget(q);
    }

    void PoseTargetCallback(const Pose_Msg::SharedPtr pose) {
        goToPoseTarget(pose->x, pose->y, pose->z, 
                        pose->roll, pose->pitch, pose->yaw, pose->cartesian);
    }

    void PublisherCallback() {
        if (planned_) {
            auto traj = most_recent_plan_.trajectory;
            auto start_state = most_recent_plan_.start_state;
            auto planning_time = PlanTime();
            planning_time.data = most_recent_plan_.planning_time;

            robot_traj_pub_->publish(traj);
            robot_start_state_pub_->publish(start_state);
            robot_plan_time_pub_->publish(planning_time);
        }
    }



    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<String_Msg>::SharedPtr named_target_sub_;
    rclcpp::Subscription<Joint_Arr>::SharedPtr joint_target_sub_; 
    rclcpp::Subscription<Pose_Msg>::SharedPtr pose_target_sub_; 

    rclcpp::Publisher<RobotTraj>::SharedPtr robot_traj_pub_;
    rclcpp::Publisher<StartState>::SharedPtr robot_start_state_pub_;
    rclcpp::Publisher<PlanTime>::SharedPtr robot_plan_time_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    MoveGroupInterface::Plan most_recent_plan_;
    bool planned_;


};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}