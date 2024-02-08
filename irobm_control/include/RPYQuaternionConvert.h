#ifndef _RPYTOQUATERNIONCONVERT_H_
#define _RPYTOQUATERNIONCONVERT_H_

#include <Eigen/Dense>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>

// const double tau = 2*M_PI;
geometry_msgs::Quaternion RPYToQuaternion(double roll, double pitch, double yaw);
Eigen::Vector3d QuaternionToRPY(geometry_msgs::Quaternion &q_g);
geometry_msgs::Quaternion RPYToQuaternion(Eigen::Vector3d &p);

#endif