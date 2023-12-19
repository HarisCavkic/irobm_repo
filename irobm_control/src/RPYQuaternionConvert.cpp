#include "RPYQuaternionConvert.h"

geometry_msgs::Quaternion RPYToQuaternion(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setEulerZYX(yaw, pitch, roll);
    geometry_msgs::Quaternion q_g;
    q_g.w = q.getW();
    q_g.x = q.getX();
    q_g.y = q.getY();
    q_g.z = q.getZ();
    return q_g;
}

Eigen::Vector3d QuaternionToRPY(geometry_msgs::Quaternion &q_g)
{
    Eigen::Quaterniond q_e(q_g.w, q_g.x, q_g.y, q_g.z);
    Eigen::Vector3d euler_angle = q_e.toRotationMatrix().eulerAngles(0, 1, 2);
    // double x = euler_angle[];
    return euler_angle;
}
