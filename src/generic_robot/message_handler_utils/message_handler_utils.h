#ifndef MESSAGE_HANDLER_UTILS_H
#define MESSAGE_HANDLER_UTILS_H

#include <ros/ros.h>
#include <rtabmap_ros/OdomInfo.h>
#include <multi_robot_separators/ReceiveSeparators.h>
#include <multi_robot_separators/PoseEstimates.h>
#include "../../slam/buzz_slam_singleton.h"

void transform_to_pose3(const geometry_msgs::Transform &msg, gtsam::Pose3 &pose3_out)
{
    gtsam::Rot3 rot(msg.rotation.w,msg.rotation.x,msg.rotation.y,msg.rotation.z);
    gtsam::Point3 pt(msg.translation.x, msg.translation.y, msg.translation.z);
    pose3_out = gtsam::Pose3(rot,pt);
}

void covariance_to_matrix(const boost::array<double, 36ul> &msg, gtsam::Matrix &cov_mat_out)
{
    cov_mat_out = gtsam::zeros(6,6);
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

void pose_with_covariance_to_msg(const graph_utils::PoseWithCovariance& pose, geometry_msgs::PoseWithCovariance &msg)
{
    msg.pose.position.x = pose.pose.x();
    msg.pose.position.y = pose.pose.y();
    msg.pose.position.z = pose.pose.z();

    gtsam::Vector quaternion = pose.pose.rotation().quaternion();
    msg.pose.orientation.w = quaternion(0);
    msg.pose.orientation.x = quaternion(1);
    msg.pose.orientation.y = quaternion(2);
    msg.pose.orientation.z = quaternion(3);

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            msg.covariance[i*6 + j] = pose.covariance_matrix(i, j);
        }
    }
}

void pose_with_covariance_from_msg(const geometry_msgs::PoseWithCovariance& msg, graph_utils::PoseWithCovariance &pose)
{
    gtsam::Rot3 rotation(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
    gtsam::Point3 translation(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    
    pose.pose = gtsam::Pose3(rotation, translation);

    pose.covariance_matrix = gtsam::zeros(6,6);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            pose.covariance_matrix(i, j) = msg.covariance[i*6 + j];
        }
    }
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