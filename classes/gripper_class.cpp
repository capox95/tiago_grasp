#include "../include/gripper_class.h"

bool GripperControl::graspAction(double value)
{
    std::vector<double> current_joints = move_group_->getCurrentJointValues();

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
    if (graspAction(0.4) == true)
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