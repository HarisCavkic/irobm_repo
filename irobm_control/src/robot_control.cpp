#include "robot_control.h"

const std::string ROBOT_CONTROL::PLANNING_GROUP = "panda_arm";

// void ROBOT_CONTROL::init_node(int argc, char **argv, const std::string &str)
// {
//     ros::init(argc, argv, str);
//     ros::NodeHandle nh;
//     ros::AsyncSpinner spinner(1);
//     spinner.start();
// }

ROBOT_CONTROL::ROBOT_CONTROL()
{
    ROBOT_CONTROL::move_group_interface = new moveit::planning_interface::MoveGroupInterface(ROBOT_CONTROL::PLANNING_GROUP);
    ROBOT_CONTROL::joint_model_group = new moveit::core::JointModelGroup(*move_group_interface->getCurrentState()->getJointModelGroup(ROBOT_CONTROL::PLANNING_GROUP));
    ROBOT_CONTROL::current_pose = ROBOT_CONTROL::move_group_interface->getCurrentPose().pose;
    geometry_msgs::Quaternion qg = RPYToQuaternion(tau/2, 0, -tau/8);
    ROBOT_CONTROL::origin_pose.orientation = qg;
    ROBOT_CONTROL::origin_pose.position.x = 0.307;
    ROBOT_CONTROL::origin_pose.position.y = 0;
    ROBOT_CONTROL::origin_pose.position.z = 1.376;
    ros::service::waitForService("/gazebo/set_model_state");
}

void ROBOT_CONTROL::set_origin_pose()
{
    ROBOT_CONTROL::planning_pose_goal(ROBOT_CONTROL::origin_pose);
}

void ROBOT_CONTROL::planning_pose_goal(geometry_msgs::Pose &p)
{
    ROBOT_CONTROL::move_group_interface->setPoseTarget(p);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ROBOT_CONTROL::move_robot(my_plan);
    ROBOT_CONTROL::current_pose = ROBOT_CONTROL::move_group_interface->getCurrentPose().pose;
}

void ROBOT_CONTROL::planning_joint_goal(std::vector<double> &joint_group_position)
{
    ROBOT_CONTROL::move_group_interface->setJointValueTarget(joint_group_position);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    ROBOT_CONTROL::move_robot(my_plan);
    ROBOT_CONTROL::current_pose = ROBOT_CONTROL::move_group_interface->getCurrentPose().pose;
}

double ROBOT_CONTROL::planning_certain_path(std::vector<geometry_msgs::Pose> &waypoints, double eef_step, double jump_threshold)
{
    moveit_msgs::RobotTrajectory traj;
    double fraction = ROBOT_CONTROL::move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, traj);
    ROBOT_CONTROL::move_group_interface->setPoseTarget(waypoints.back());
    ROBOT_CONTROL::move_robot(traj);
    ROBOT_CONTROL::current_pose = ROBOT_CONTROL::move_group_interface->getCurrentPose().pose;
    return fraction;
}

geometry_msgs::Pose ROBOT_CONTROL::get_origin_pose() const
{
    return ROBOT_CONTROL::origin_pose;
}

geometry_msgs::Pose ROBOT_CONTROL::get_current_pose() const
{
    return ROBOT_CONTROL::current_pose;
}

void ROBOT_CONTROL::print_current_pose() const
{
    geometry_msgs::Quaternion q = ROBOT_CONTROL::get_current_pose().orientation;
    Eigen::Vector3d rpy = QuaternionToRPY(q)/(tau/360);
    ROS_INFO("orientation: (%f, %f, %f, %f)\neuler angle(yzx): (%f, %f, %f)\nposition: (%f, %f, %f)", ROBOT_CONTROL::current_pose.orientation.w, 
            ROBOT_CONTROL::current_pose.orientation.x, ROBOT_CONTROL::current_pose.orientation.y, 
            ROBOT_CONTROL::current_pose.orientation.z, rpy[0], rpy[1], rpy[2], ROBOT_CONTROL::current_pose.position.x, ROBOT_CONTROL::current_pose.position.y, 
            ROBOT_CONTROL::current_pose.position.z);
}

void ROBOT_CONTROL::move_robot(moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    bool success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "moving robot %s", success ? "" : "FAILED");
    ROBOT_CONTROL::move_group_interface->execute(plan);
    ROBOT_CONTROL::move_group_interface->move();
}

void ROBOT_CONTROL::move_robot(moveit_msgs::RobotTrajectory &trajectory)
{
    ROBOT_CONTROL::move_group_interface->execute(trajectory);
    ROBOT_CONTROL::move_group_interface->move();
}

ROBOT_CONTROL::~ROBOT_CONTROL()
{
    if(ROBOT_CONTROL::joint_model_group != nullptr)
    {
        delete ROBOT_CONTROL::joint_model_group;
        ROBOT_CONTROL::joint_model_group = nullptr;
    }
    if(ROBOT_CONTROL::move_group_interface != nullptr)
    {
        delete ROBOT_CONTROL::move_group_interface;
        ROBOT_CONTROL::move_group_interface = nullptr;
    }
}

double ROBOT_CONTROL::arc_path(geometry_msgs::Point &center_of_circle, double radius, int times)
{
    geometry_msgs::Pose p = ROBOT_CONTROL::current_pose;
    p.position.z = center_of_circle.z + radius + 0.05;
    p.position.y = center_of_circle.y + radius;
    p.position.x = center_of_circle.x;
    Eigen::Vector3d rpy_o = QuaternionToRPY(ROBOT_CONTROL::origin_pose.orientation);
    Eigen::Vector3d rpy(rpy_o);
    rpy[2] -= tau/4;
    p.orientation = RPYToQuaternion(rpy);
    ROBOT_CONTROL::planning_pose_goal(p);
    // system("pause");
    // p = ROBOT_CONTROL::current_pose;
    // Eigen::Vector3d rpy_o = QuaternionToRPY(ROBOT_CONTROL::origin_pose.orientation);
    // double a = atan2(center_of_circle.y - p.position.y, center_of_circle.x - p.position.x);
    // Eigen::Vector3d rpy(rpy_o);
    // rpy[2] += a;
    // p.orientation = RPYToQuaternion(rpy);
    // ROBOT_CONTROL::planning_pose_goal(p);
    // system("pause");
    p = ROBOT_CONTROL::current_pose;
    std::vector<geometry_msgs::Pose> waypoints;
    for(int i{0};i < times;i++)
    {
        rpy = rpy_o;
        rpy[2] += -tau/4 + tau/24*(i+1);
        p.orientation = RPYToQuaternion(rpy);
        p.position.x = center_of_circle.x + radius * cos(tau/2 - tau/4 + tau/24*(i+1));
        p.position.y = center_of_circle.y + radius * sin(tau/2 - tau/4 + tau/24*(i+1));
        waypoints.push_back(p);
    }
    waypoints.pop_back();
    double fraction = ROBOT_CONTROL::planning_certain_path(waypoints);
    return fraction;
}
