#include "robot_control.h"
#include <RPYQuaternionConvert.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_robot");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROBOT_CONTROL RC;
    geometry_msgs::Pose pos_o = RC.get_origin_pose();
    geometry_msgs::Pose pos_target;
    Eigen::Vector3d rpy;
    double x,y,z;
    
    while(ros::ok())
    {
        ROS_INFO("please select your choice!");
        ROS_INFO("1. move robot to a point simply.");
        ROS_INFO("2. move the robot to a certain joint space.");
        ROS_INFO("3. move the robot like a arc.");
        ROS_INFO("4. move the robot self-defined trajectory.");
        ROS_INFO("5. set the robot to the original position.");
        ROS_INFO("6. get the current pose of the robot. ");
        ROS_INFO("0. quit the operation.");

        int choice;
        std::cin >> choice;
        switch(choice)
        {
            case 1:
            {
                pos_target = RC.get_current_pose();
                ROS_INFO("Enter the pose you want: \n");
                double euler_angle_z;
                do
                {
                    ROS_INFO("Enter the euler angle which the robot needs to rotate the z-axis: ");
                    std::cin >> euler_angle_z;
                    
                }while(std::cin.fail() && std::cout << "Enter failed, try again!");
                rpy = QuaternionToRPY(pos_target.orientation);
                rpy[2] = euler_angle_z-tau/8;
                pos_target.orientation = RPYToQuaternion(rpy);

                do
                {
                    ROS_INFO("Enter the distance along the x-axis: ");
                    std::cin >> x;
                }while(std::cin.fail() && std::cout << "Enter failed, try again!");
                pos_target.position.x = x;
                
                do
                {
                    ROS_INFO("Enter the distance along the y-axis: ");
                    std::cin >> y;
                }while(std::cin.fail() && std::cout << "Enter failed, try again!");
                pos_target.position.y = y;
                
                do
                {
                    ROS_INFO("Enter the distance along the z-axis: ");
                    std::cin >> z;
                }while(std::cin.fail() && std::cout << "Enter failed, try again!");

                pos_target.position.z = z;
                RC.planning_pose_goal(pos_target);
                // system("pause");
                break;
            }

            // case 2:
            // break;

            case 3:
            {
                geometry_msgs::Point c;
                do
                {
                    ROS_INFO("Enter center point coordinate along x-axis: ");
                    std::cin >> c.x;
                }while(std::cin.fail() && std::cout << "Enter failed, try again!");

                do
                {
                    ROS_INFO("Enter center point coordinate along y-axis: ");
                    std::cin >> c.y;
                }while(std::cin.fail() && std::cout << "Enter failed, try again!");

                do
                {
                    ROS_INFO("Enter center point coordinate along z-axis: ");
                    std::cin >> c.z;
                }while(std::cin.fail() && std::cout << "Enter failed, try again!");
                c.z += 0.787;

                double radius;
                int times;

                do
                {
                    ROS_INFO("Enter the radius: ");
                    std::cin >> radius;
                }while(std::cin.fail() && std::cout << "Enter failed, try again!");

                do
                {
                    ROS_INFO("Enter the number of waypoints, each waypoint rotates the center point 15 degrees: ");
                    std::cin >> times;
                    times %= 24;
                }while(std::cin.fail() && std::cout << "Enter failed, try again!");

                double fraction = RC.arc_path(c, radius, times);
                // system("pause");
                break;
            }

            // case 4:
            // break;

            case 5:
            RC.set_origin_pose();
            break;

            case 6:
            RC.print_current_pose();
            // system("pause");
            break;

            case 0:
            {
                ROS_INFO("welcome to next use!");
                ros::shutdown();
                // system("pause");
                return 0;
                break;
            }

            default:
            ROS_INFO("your choice is default. Please try again!");
            break;
        }
    }
    // ros::shutdown();
    // return 0;
}