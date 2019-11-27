#include <memory>

#include <ros/ros.h>

#include <pluginlib/class_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

class ArmControl
{
private:
    ros::NodeHandle nh_;
    std::string planning_group_name_;
    double POS_TOLARENCE, ANG_TOLARENCE, PLANING_TIME;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    robot_model::RobotModelPtr kinematic_model_;
    kinematics::KinematicsBasePtr kinematics_solver_;
    std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> kinematics_loader_;
    std::string plugin_name_;
    std::string robot_description_;
    const robot_state::JointModelGroup *joint_model_group_;
    std::string world_frame_;
    std::string ee_frame_;

    std::unique_ptr<planning_scene::PlanningScene> planning_scene_;
    ros::ServiceClient planning_scene_diff_client_;

    geometry_msgs::Pose current_pose_;
    moveit_msgs::PlanningScene planning_scene_msg_;
    collision_detection::AllowedCollisionMatrix acm_;
    moveit_msgs::ApplyPlanningScene srv_;

    //moveit_msgs::CollisionObject object_;
    //moveit_msgs::AllowedCollisionMatrix acm_msg_;

    float dim_box_, offset_gripper_tip_;

public:
    ArmControl() : nh_(),
                   planning_group_name_("arm_torso"),
                   POS_TOLARENCE(0.04),
                   ANG_TOLARENCE(0.1),
                   PLANING_TIME(20.0),
                   plugin_name_("kdl_kinematics_plugin/KDLKinematicsPlugin"),
                   robot_description_("robot_description"),
                   world_frame_("base_footprint"),
                   ee_frame_("arm_tool_link")
    {

        move_group_.reset(new moveit::planning_interface::MoveGroupInterface(planning_group_name_));
        move_group_->setPlanningTime(PLANING_TIME);
        move_group_->setGoalOrientationTolerance(ANG_TOLARENCE);
        move_group_->setGoalPositionTolerance(POS_TOLARENCE);
        ROS_INFO("Move_Group_ Ready!");

        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        kinematic_model_ = robot_model_loader.getModel();
        joint_model_group_ = move_group_->getCurrentState()->getJointModelGroup("arm_torso");

        planning_scene_.reset(new planning_scene::PlanningScene(kinematic_model_));
        planning_scene_diff_client_ = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
        planning_scene_diff_client_.waitForExistence();

        dim_box_ = 0.1;             //used to perform grasp action, should be related to the size of the gripper
        offset_gripper_tip_ = 0.22; //offset between arm_tool_link (ee of arm-torso group) and gripper tips
    }

    // compute cartesian trajectory. Straight path along "z axis" for a distance of "value". Reference point is goal_pose;
    double computeCartesianTrajectoryPostGrasp(geometry_msgs::Pose current, double value, moveit_msgs::RobotTrajectory &trajectory);

    double computeCartesianTrajectoryFromStartState(std::vector<std::string> &joint_names,
                                                    std::vector<double> &start_joints,
                                                    geometry_msgs::Pose &pose,
                                                    moveit_msgs::RobotTrajectory &trajectory);

    bool preGraspApproach(geometry_msgs::Pose goal, double approaching_distance);

    bool postGraspApproach(geometry_msgs::Pose goal, double departure_distance);

    // MOVE ARM TO GOAL POSE
    bool moveToPose(geometry_msgs::Pose goal);

    void constructCollisionObject(moveit_msgs::CollisionObject &object, geometry_msgs::Pose &pose,
                                  moveit_msgs::AllowedCollisionMatrix &acm_msg);

    void publishCollisionObject(moveit_msgs::CollisionObject &object, moveit_msgs::AllowedCollisionMatrix &acm_msg);

    void removeCollisionObject(moveit_msgs::CollisionObject &object, moveit_msgs::AllowedCollisionMatrix &acm_msg);
};