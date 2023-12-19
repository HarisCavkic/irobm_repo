#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <Eigen/Geometry>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include "RPYQuaternionConvert.h"

class ROBOT_CONTROL
{
    public:
    static const std::string PLANNING_GROUP;
    moveit::planning_interface::MoveGroupInterface *move_group_interface;
    const moveit::core::JointModelGroup* joint_model_group;
    // static void init_node(int argc, char **argv, const std::string &str);
    
    ROBOT_CONTROL();

    ~ROBOT_CONTROL();

    void set_origin_pose();

    void planning_pose_goal(geometry_msgs::Pose &p);

    void planning_joint_goal(std::vector<double> &joint_group_position);

    double planning_certain_path(std::vector<geometry_msgs::Pose> &waypoints, double eef_step = 0.01, double jump_threshold = 0);

    geometry_msgs::Pose get_current_pose() const;

    geometry_msgs::Pose get_origin_pose() const;

    void print_current_pose() const;

    double arc_path(geometry_msgs::Point &center_of_circle, double radius = 0.12, int times = 14);
    
    private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    geometry_msgs::Pose origin_pose;
    geometry_msgs::Pose current_pose;

    void move_robot(moveit::planning_interface::MoveGroupInterface::Plan &plan);

    void move_robot(moveit_msgs::RobotTrajectory &trajectory);

};

const double tau = 2 * M_PI;

#endif