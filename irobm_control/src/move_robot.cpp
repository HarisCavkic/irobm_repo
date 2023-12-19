#include "robot_control.h"
#include <RPYQuaternionConvert.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_robot");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROBOT_CONTROL RC;
    RC.print_current_pose();
    RC.set_origin_pose();
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose p = RC.get_current_pose();

    // p.position.x += 0.1;
    // waypoints.push_back(p);
    // p.position.y += 0.1;
    // waypoints.push_back(p);
    // p.position.z += 0.1;
    // waypoints.push_back(p);
    // double fraction = RC.planning_certain_path(waypoints);

    geometry_msgs::Point c;
    c.x = 0.5;
    c.y = 0;
    c.z = 0.79;
    RC.arc_path(c);

    // geometry_msgs::Pose p = RC.get_current_pose();
    // p.position.x = c.x;
    // p.position.y = c.y + 0.12;
    // p.orientation = RPYToQuaternion(tau/2, 0, -tau/8*3);
    // RC.planning_pose_goal(p);
    
    // while(ros::ok())
    // {
    //     ROS_INFO("please select your choice!");
    //     ROS_INFO("1. move robot to a point simply.");
    //     ROS_INFO("2. move the robot to a certain joint space.");
    //     ROS_INFO("3. move the robot like a arc.");
    //     ROS_INFO("0. quit the operation.");

    //     int choice;
    //     std::cin >> choice;
    //     switch(choice)
    //     {
    //         case 1:
    //         break;

    //         case 2:
    //         break;

    //         case 3:
    //         break;

    //         case 0:
    //         ROS_INFO("welcome to next use!");
    //         ros::shutdown();
    //         return 0;
    //         break;

    //         default:
    //         ROS_INFO("your choice is default. Please try again!");
    //         break;
    //     }
    // }
    ros::shutdown();
    return 0;
}