#ifndef MESSAGE_HANDLER_UTILS_H
#define MESSAGE_HANDLER_UTILS_H

#include <ros/ros.h>
#include <rtabmap_ros/OdomInfo.h>
#include <multi_robot_separators/ReceiveSeparators.h>

void transform_to_pose3(const geometry_msgs::Transform &msg, gtsam::Pose3 &pose3_out)
{
    gtsam::Rot3 rot(msg.rotation.w,msg.rotation.x,msg.rotation.y,msg.rotation.z);
    gtsam::Point3 pt(msg.translation.x, msg.translation.y, msg.translation.z);
    pose3_out = gtsam::Pose3(rot,pt);
}

void covariance_to_matrix(const boost::array<double, 36ul> &msg, gtsam::Matrix &cov_mat_out)
{
    for (int row = 0; row < 6; row++)
    {
        for (int col = 0; col < 6; col++)
        {
            cov_mat_out(row,col) = msg[col+row*6];
        }
    }
}

void pose_ros_to_gtsam(const geometry_msgs::Pose &msg, gtsam::Pose3 &pose3_out)
{
    gtsam::Rot3 rot(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    gtsam::Point3 pt(msg.position.x, msg.position.y, msg.position.z);
    pose3_out = gtsam::Pose3(rot, pt);
}

void set_covariance_matrix(gtsam::Matrix &covariance_matrix, const double& rotation_std, const double& translation_std)
{
    covariance_matrix = gtsam::zeros(6,6);
    covariance_matrix(0, 0) = rotation_std * rotation_std;
    covariance_matrix(1, 1) = rotation_std * rotation_std;
    covariance_matrix(2, 2) = rotation_std * rotation_std;
    covariance_matrix(3, 3) = translation_std * translation_std;
    covariance_matrix(4, 4) = translation_std * translation_std;
    covariance_matrix(5, 5) = translation_std * translation_std;
}

#endif