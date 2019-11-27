#include "../include/gripper_class.h"

bool GripperControl::graspAction(double value)
{
    std::vector<double> current_joints = move_group_->getCurrentJointValues();

    //------------------------------------------------------------------------
    moveit_msgs::AllowedCollisionMatrix acm_msg;

    planning_scene_->getAllowedCollisionMatrixNonConst().setEntry("<octomap>", true);
    acm_ = planning_scene_->getAllowedCollisionMatrix();

    // obtain acm matrix as moveit message for service
    acm_.getMessage(acm_msg);

    // publish object to planning scene
    publishCollisionObject(acm_msg);
    //------------------------------------------------------------------------

    std::vector<std::string> joints_name = move_group_->getJointNames();
    for (int i = 0; i < joints_name.size(); i++)
    {
        if (gripper_left_.compare(joints_name[i]) == 0)
        {
            current_joints[i] = value;
        }
        if (gripper_right_.compare(joints_name[i]) == 0)
        {
            current_joints[i] = value;
        }
    }

    move_group_->setStartStateToCurrentState();
    move_group_->setJointValueTarget(current_joints);

    //move_group_->setGoalTolerance(0.05);

    bool success = (move_group_->plan(plan_hand_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
        return false;
    }
    bool execution = (move_group_->execute(plan_hand_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (execution)
        return true;
    else
        return false;
}

bool GripperControl::openGripper()
{
    if (graspAction(0.38) == true)
        return true;
    else
        return false;
}

bool GripperControl::closeGripper()
{
    if (graspAction(0.0) == true)
        return true;
    else
        return false;
}

bool GripperControl::moveToJointValue(double value)
{
    if (graspAction(value) == true)
        return true;
    else
        return false;
}

void GripperControl::publishCollisionObject(moveit_msgs::AllowedCollisionMatrix &acm_msg)
{
    planning_scene_msg_.is_diff = true;
    planning_scene_msg_.robot_state.is_diff = true;
    planning_scene_msg_.allowed_collision_matrix = acm_msg;

    srv_.request.scene = planning_scene_msg_;
    planning_scene_diff_client_.call(srv_);
}
