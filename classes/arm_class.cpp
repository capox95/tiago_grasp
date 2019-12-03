#include "../include/arm_class.h"

// compute cartesian trajectory. Straight path along "z axis" for a distance of "value". Reference point is goal_pose;
double ArmControl::computeCartesianTrajectoryPostGrasp(geometry_msgs::Pose current, double value, moveit_msgs::RobotTrajectory &trajectory)
{
    robot_state::RobotStatePtr start_state = move_group_->getCurrentState();
    move_group_->setStartState(*start_state);

    /*
    // -------------------------------------------------------------------------------
    // create collision object and publish it
    moveit_msgs::CollisionObject object;
    moveit_msgs::AllowedCollisionMatrix acm_msg;
    constructCollisionObject(object, current, acm_msg);
    // -----------------------------------------------------------------------------
    */

    std::vector<geometry_msgs::Pose> waypoints;
    current.position.z += value;
    waypoints.push_back(current);

    //move_group_->setMaxVelocityScalingFactor(0.05);
    //move_group_->setMaxAccelerationScalingFactor(0.05);

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, false);
    if (fraction < 1.0)
        ROS_INFO("fraction < 1.0; value: %f", fraction);

    /*
    // -----------------------------------------------------------------------------
    // remove collision object
    removeCollisionObject(object, acm_msg);
    // -----------------------------------------------------------------------------
    */

    return fraction;
}

double ArmControl::computeCartesianTrajectoryFromStartState(std::vector<std::string> &joint_names,
                                                            std::vector<double> &start_joints,
                                                            geometry_msgs::Pose &pose,
                                                            moveit_msgs::RobotTrajectory &trajectory)
{
    // set the start state
    robot_state::RobotStatePtr start_state = move_group_->getCurrentState();
    start_state->setJointGroupPositions(joint_model_group_, start_joints);
    move_group_->setStartState(*start_state);

    // add cartesian waypoint
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(pose);

    //move_group_->setMaxVelocityScalingFactor(0.05);
    //move_group_->setMaxAccelerationScalingFactor(0.05);

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, false);
    if (fraction < 1.0)
        ROS_INFO("fraction < 1.0; value: %f", fraction);

    return fraction;
}

bool ArmControl::preGraspApproach(geometry_msgs::Pose goal, double approaching_distance)
{

    moveit::planning_interface::MoveGroupInterface::Plan approach, cartesian;

    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);

    robot_state::RobotStatePtr start_state = move_group_->getCurrentState();
    move_group_->setStartState(*start_state);

    geometry_msgs::Pose goal_approach = goal;
    goal_approach.position.z += approaching_distance;
    move_group_->setPoseTarget(goal_approach);

    bool approach_plan = move_group_->plan(approach) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if (!approach_plan)
        return false;

    std::size_t traj_size = approach.trajectory_.joint_trajectory.points.size();
    trajectory_msgs::JointTrajectoryPoint jtp = approach.trajectory_.joint_trajectory.points.at(traj_size - 1);
    std::vector<double> joint_values = jtp.positions;
    std::vector<std::string> joint_names = approach.trajectory_.joint_trajectory.joint_names;

    /*
    // -------------------------------------------------------------------------------
    // create collision object and publish it
    moveit_msgs::CollisionObject object;
    moveit_msgs::AllowedCollisionMatrix acm_msg;
    constructCollisionObject(object, goal, acm_msg);
    // -----------------------------------------------------------------------------
    */

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = computeCartesianTrajectoryFromStartState(joint_names, joint_values, goal, trajectory);
    if (trajectory.joint_trajectory.points.size() <= 1 || fraction < 1.0)
        return false;
    else
        cartesian.trajectory_ = trajectory;

    // executions --------------------------------------------------------------------------------------------------

    bool success_approach = move_group_->execute(approach) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if (!success_approach)
        return false;

    bool success_cartesian = move_group_->execute(cartesian) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if (!success_cartesian)
        return false;

    current_pose_ = goal;

    /*
    // -----------------------------------------------------------------------------
    // remove collision object
    removeCollisionObject(object, acm_msg);
    // -----------------------------------------------------------------------------
    */

    return true;
}

bool ArmControl::postGraspApproach(geometry_msgs::Pose goal, double departure_distance)
{
    moveit::planning_interface::MoveGroupInterface::Plan cartesian, departure;

    geometry_msgs::Pose current = current_pose_;

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = computeCartesianTrajectoryPostGrasp(current, departure_distance, trajectory);
    if (trajectory.joint_trajectory.points.size() <= 1 || fraction < 0.8)
        return false;
    else
        cartesian.trajectory_ = trajectory;

    move_group_->clearPathConstraints();
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);

    std::size_t traj_size = cartesian.trajectory_.joint_trajectory.points.size();
    trajectory_msgs::JointTrajectoryPoint jtp = cartesian.trajectory_.joint_trajectory.points.at(traj_size - 1);
    std::vector<double> departure_joints = jtp.positions;

    robot_state::RobotStatePtr start_state = move_group_->getCurrentState();
    start_state->setJointGroupPositions(joint_model_group_, departure_joints);
    move_group_->setStartState(*start_state);

    move_group_->setPoseTarget(goal);

    bool departure_success = move_group_->plan(departure) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if (!departure_success)
        return false;

    //std::cout << "plan departure" << std::endl;
    //
    // executions --------------------------------------------------------------------------------------------------
    bool success_cartesian = move_group_->execute(cartesian) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if (!success_cartesian)
        return false;

    bool success_departure = move_group_->execute(departure) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if (!success_departure)
        return false;

    return true;
}

// MOVE ARM TO GOAL POSE
bool ArmControl::moveToPose(geometry_msgs::Pose goal)
{
    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);

    move_group_->setPoseTarget(goal);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::FAILURE)
        return false;

    if (move_group_->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        return true;
    else
        false;
}

void ArmControl::constructCollisionObject(moveit_msgs::CollisionObject &object, geometry_msgs::Pose &pose,
                                          moveit_msgs::AllowedCollisionMatrix &acm_msg)
{

    object.header.frame_id = world_frame_;
    object.id = "boxBig";

    geometry_msgs::Pose box_pose;
    box_pose.position.x = pose.position.x;
    box_pose.position.y = pose.position.y;
    box_pose.position.z = pose.position.z - offset_gripper_tip_;
    box_pose.orientation.w = 1.0;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = dim_box_ * 2;
    primitive.dimensions[1] = dim_box_ * 2;
    primitive.dimensions[2] = dim_box_ * 2;

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(box_pose);
    object.operation = object.ADD;

    planning_scene_->getAllowedCollisionMatrixNonConst().setEntry(object.id, true);
    acm_ = planning_scene_->getAllowedCollisionMatrix();

    // obtain acm matrix as moveit message for service
    acm_.getMessage(acm_msg);

    // publish object to planning scene
    publishCollisionObject(object, acm_msg);
}

void ArmControl::publishCollisionObject(moveit_msgs::CollisionObject &object, moveit_msgs::AllowedCollisionMatrix &acm_msg)
{
    planning_scene_msg_.world.collision_objects.clear();
    planning_scene_msg_.world.collision_objects.push_back(object);
    planning_scene_msg_.is_diff = true;
    planning_scene_msg_.robot_state.is_diff = true;
    planning_scene_msg_.allowed_collision_matrix = acm_msg;

    srv_.request.scene = planning_scene_msg_;
    planning_scene_diff_client_.call(srv_);
}

void ArmControl::removeCollisionObject(moveit_msgs::CollisionObject &object, moveit_msgs::AllowedCollisionMatrix &acm_msg)
{

    object.operation = object.REMOVE;

    planning_scene_->getAllowedCollisionMatrixNonConst().removeEntry(object.id);
    acm_ = planning_scene_->getAllowedCollisionMatrix();

    // obtain acm matrix as moveit message for service
    acm_.getMessage(acm_msg);

    publishCollisionObject(object, acm_msg);
}

// ------------------------------------------------
// CLOTHES RECOVERY -------------------------------
// ------------------------------------------------

bool ArmControl::preRecoveryApproach(geometry_msgs::Pose goal, double cartesian_distance)
{
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");
    visual_tools.loadRemoteControl();

    ROS_WARN("GOAL: %f, %f, %f", goal.position.x, goal.position.y, goal.position.z);

    moveit::planning_interface::MoveGroupInterface::Plan approach, cartesian;

    move_group_->setMaxVelocityScalingFactor(1.0);
    move_group_->setMaxAccelerationScalingFactor(1.0);

    robot_state::RobotStatePtr start_state = move_group_->getCurrentState();
    move_group_->setStartState(*start_state);

    geometry_msgs::Pose approach_goal = goal;
    approach_goal.position.x -= cartesian_distance;
    move_group_->setPoseTarget(approach_goal);

    bool approach_plan = move_group_->plan(approach) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if (!approach_plan)
        return false;

    std::size_t traj_size = approach.trajectory_.joint_trajectory.points.size();
    trajectory_msgs::JointTrajectoryPoint jtp = approach.trajectory_.joint_trajectory.points.at(traj_size - 1);
    std::vector<double> joint_values = jtp.positions;
    std::vector<std::string> joint_names = approach.trajectory_.joint_trajectory.joint_names;

    visual_tools.prompt("NEXT FOR CARTESIAN PLAN");
    // -----------------------------------------------------------------------------

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = computeCartesianTrajectoryFromStartState(joint_names, joint_values, goal, trajectory);
    if (trajectory.joint_trajectory.points.size() <= 1 || fraction < 1.0)
        return false;
    else
        cartesian.trajectory_ = trajectory;

    visual_tools.prompt("NEXT FOR EXECUTION");

    // ------------------------------------------ EXECUTION --------------------------------------------

    bool success_approach = move_group_->execute(approach) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if (!success_approach)
        return false;

    bool success_cartesian = move_group_->execute(cartesian) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if (!success_cartesian)
        return false;

    current_pose_ = goal;

    return true;
}

bool ArmControl::postRecoveryApproach(geometry_msgs::Pose goal, double cartesian_distance)
{
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");
    visual_tools.loadRemoteControl();

    moveit::planning_interface::MoveGroupInterface::Plan cartesian;

    geometry_msgs::Pose current = current_pose_;

    geometry_msgs::Pose goal2 = goal;

    moveit_msgs::RobotTrajectory trajectory;
    robot_state::RobotStatePtr start_state = move_group_->getCurrentState();
    move_group_->setStartState(*start_state);

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(goal);

    goal2.position.x += 0.1;
    waypoints.push_back(goal2);

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, false);
    if (fraction < 1.0)
    {
        ROS_INFO("fraction < 1.0; value: %f", fraction);
        return false;
    }

    cartesian.trajectory_ = trajectory;
    visual_tools.prompt("NEXT FOR EXECUTION");

    // executions --------------------------------------------------------------------------------------------------

    bool success_cartesian = move_group_->execute(cartesian) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if (!success_cartesian)
        return false;

    return true;
}
