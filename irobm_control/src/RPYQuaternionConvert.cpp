#include "RPYQuaternionConvert.h"

geometry_msgs::Quaternion RPYToQuaternion(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    geometry_msgs::Quaternion q_g;
    tf2::convert(q, q_g);
    return q_g;
}

geometry_msgs::Quaternion RPYToQuaternion(Eigen::Vector3d &p)
{
    tf2::Quaternion q;
    q.setRPY(p[0], p[1], p[2]);
    geometry_msgs::Quaternion q_g;
    tf2::convert(q, q_g);
    return q_g;
}

Eigen::Vector3d QuaternionToRPY(geometry_msgs::Quaternion &q_g)
{
    Eigen::Quaterniond q_e(q_g.w, q_g.x, q_g.y, q_g.z);
    Eigen::Vector3d euler_angle;
    tf2::Quaternion q;
    tf2::convert(q_g, q);
    tf2::Matrix3x3 rotate_matrix(q);
    rotate_matrix.getRPY(euler_angle[0], euler_angle[1], euler_angle[2]);
    // double x = euler_angle[];
    return euler_angle;
}
