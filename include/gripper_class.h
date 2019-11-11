#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>

class GripperControl
{
private:
    ros::NodeHandle nh_;
    std::string planning_group_name_;
    double POS_TOLARENCE, ANG_TOLARENCE, PLANING_TIME;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    std::vector<double> joints_goal_;
    moveit::planning_interface::MoveGroupInterface::Plan plan_hand_;
    std::string gripper_left_, gripper_right_;

public:
    GripperControl() : nh_("~"),
                       planning_group_name_("gripper"),
                       POS_TOLARENCE(0.01),
                       ANG_TOLARENCE(0.1),
                       PLANING_TIME(5.0),
                       gripper_left_("gripper_left_finger_joint"),
                       gripper_right_("gripper_right_finger_joint")
    {
        move_group_.reset(new moveit::planning_interface::MoveGroupInterface(planning_group_name_));
        move_group_->setPlanningTime(PLANING_TIME);
        move_group_->setGoalOrientationTolerance(ANG_TOLARENCE);
        move_group_->setGoalPositionTolerance(POS_TOLARENCE);
        ROS_INFO("Move_Group_ Ready!");
    }

    GripperControl(std::string planning_group, std::string gripper_left, std::string gripper_right) : nh_("~"),
                                                                                                      planning_group_name_(planning_group),
                                                                                                      POS_TOLARENCE(0.01),
                                                                                                      ANG_TOLARENCE(0.1),
                                                                                                      PLANING_TIME(5.0),
                                                                                                      gripper_left_(gripper_left),
                                                                                                      gripper_right_(gripper_right)
    {
        move_group_.reset(new moveit::planning_interface::MoveGroupInterface(planning_group_name_));
        move_group_->setPlanningTime(PLANING_TIME);
        move_group_->setGoalOrientationTolerance(ANG_TOLARENCE);
        move_group_->setGoalPositionTolerance(POS_TOLARENCE);
        ROS_INFO("Move_Group_ Ready!");
    }

    bool openGripper();

    bool closeGripper();

    bool moveToJointValue(double value);

private:
    bool graspAction(double value);
};