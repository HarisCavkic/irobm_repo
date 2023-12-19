#include "robot_control.h"
#include <RPYQuaternionConvert.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_robot");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROBOT_CONTROL RC;
    geometry_msgs::Quaternion qg = RPYToQuaternion(0, tau/2, -tau/8*3);
    geometry_msgs::Pose target_pose;
    target_pose.orientation = qg;
    target_pose.position.x = 0.307;
    target_pose.position.y = 0;
    target_pose.position.z = 1.376;
    // RC.planning_pose_goal(target_pose);
    // moveit::core::RobotStatePtr cs = RC.move_group_interface->getCurrentState();
    // std::vector<double> joint_gs;
    // cs->copyJointGroupPositions(RC.joint_model_group, joint_gs);
    // joint_gs[0] = -tau/6;
    // RC.planning_joint_goal(joint_gs);
    RC.print_current_pose();
    ros::shutdown();
    return 0;
}