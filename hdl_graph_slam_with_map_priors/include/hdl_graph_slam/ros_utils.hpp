// SPDX-License-Identifier: BSD-2-Clause

#ifndef ROS_UTILS_HPP
#define ROS_UTILS_HPP

#include <Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

namespace hdl_graph_slam {

/**
 * @brief convert Eigen::Matrix to geometry_msgs::TransformStamped
 * @param stamp            timestamp
 * @param pose             Eigen::Matrix to be converted
 * @param frame_id         tf frame_id
 * @param child_frame_id   tf child frame_id
 * @return converted TransformStamped
 */
static geometry_msgs::TransformStamped matrix2transform(const ros::Time& stamp, const Eigen::Matrix4f& pose, const std::string& frame_id, const std::string& child_frame_id) {
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  quat.normalize();
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = stamp;
  odom_trans.header.frame_id = frame_id;
  odom_trans.child_frame_id = child_frame_id;

  odom_trans.transform.translation.x = pose(0, 3);
  odom_trans.transform.translation.y = pose(1, 3);
  odom_trans.transform.translation.z = pose(2, 3);
  odom_trans.transform.rotation = odom_quat;

  return odom_trans;
}

/**
 * @brief convert Eigen::Vector2d to Eigen::Isometry2d (rotation = identity)
 */
static Eigen::Isometry2d getIsoFromTrans(Eigen::Vector2d trans) {
  Eigen::Isometry2d iso = Eigen::Isometry2d::Identity();
  iso.translation() = trans;
  return iso;
}
/**
 * @brief convert Eigen::Matrix3f to Eigen::Matrix4f (float 2d to float 3d)
 */
static Eigen::Matrix4f matrix3fto4f(Eigen::Matrix3f m3f) {
  Eigen::Matrix4f m4f = Eigen::Matrix4f::Identity();
  m4f.block<2,2>(0,0) = m3f.block<2,2>(0,0);
  m4f.block<2,1>(0,3) = m3f.block<2,1>(0,2);
  return m4f;
} 
/**
 * @brief convert Eigen::Matrix3f (2d) to geometry_msgs::TransformStamped 
 */
static geometry_msgs::TransformStamped matrix2transform2d(const ros::Time& stamp, const Eigen::Matrix3f& pose_2d, const std::string& frame_id, const std::string& child_frame_id) {
  Eigen::Matrix4f pose = matrix3fto4f(pose_2d);
  Eigen::Quaternionf quat(pose.block<3, 3>(0, 0));
  quat.normalize();
  geometry_msgs::Quaternion odom_quat;
  odom_quat.w = quat.w();
  odom_quat.x = quat.x();
  odom_quat.y = quat.y();
  odom_quat.z = quat.z();

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = stamp;
  odom_trans.header.frame_id = frame_id;
  odom_trans.child_frame_id = child_frame_id;

  odom_trans.transform.translation.x = pose(0, 3);
  odom_trans.transform.translation.y = pose(1, 3);
  odom_trans.transform.translation.z = 0;
  odom_trans.transform.rotation = odom_quat;

  return odom_trans;
}

static Eigen::Isometry3d pose2isometry(const geometry_msgs::Pose& pose) {
  Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
  mat.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  mat.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).toRotationMatrix();
  return mat;
}

static Eigen::Isometry3d tf2isometry(const tf::StampedTransform& trans) {
  Eigen::Isometry3d mat = Eigen::Isometry3d::Identity();
  mat.translation() = Eigen::Vector3d(trans.getOrigin().x(), trans.getOrigin().y(), trans.getOrigin().z());
  mat.linear() = Eigen::Quaterniond(trans.getRotation().w(), trans.getRotation().x(), trans.getRotation().y(), trans.getRotation().z()).toRotationMatrix();
  return mat;
}

static geometry_msgs::Pose isometry2pose(const Eigen::Isometry3d& mat) {
  Eigen::Quaterniond quat(mat.linear());
  Eigen::Vector3d trans = mat.translation();

  geometry_msgs::Pose pose;
  pose.position.x = trans.x();
  pose.position.y = trans.y();
  pose.position.z = trans.z();
  pose.orientation.w = quat.w();
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();

  return pose;
}

static Eigen::Isometry3d odom2isometry(const nav_msgs::OdometryConstPtr& odom_msg) {
  const auto& orientation = odom_msg->pose.pose.orientation;
  const auto& position = odom_msg->pose.pose.position;

  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;

  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.linear() = quat.toRotationMatrix();
  isometry.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return isometry;
}

/**
 * @brief convert Eigen::Quaterniond (3D) to an angle (2D) - assuming the 3D rotation is already mainly around z, the rotations around x and y are discarded and only the one around z is kept as angle
 */
static double quatToAngle(Eigen::Quaterniond quat) {
  Eigen::AngleAxisd aa = Eigen::AngleAxisd(quat);
  double angle = 0.0;
  if(aa.axis()(2) > 0)  
    angle = aa.angle();
  else
    angle = -aa.angle();
  return angle;
}
/**
 * @brief convert Eigen::Matrix3d rotation matrix (3D) to an angle (2D) - assuming the 3D rotation is already mainly around z, the rotations around x and y are discarded and only the one around z is kept as angle
 */
static double rotToAngle(Eigen::Matrix3d rot_mat) {
  Eigen::AngleAxisd aa = Eigen::AngleAxisd(rot_mat);
  double angle = 0.0;
  if(aa.axis()(2) > 0)  
    angle = aa.angle();
  else
    angle = -aa.angle();
  return angle;
}
/**
 * @brief given an Eigen::Quaterniond (3D orientation) and a Eigen::Vector3d (3D position) return the corresponding Eigen::Isometry2d (2D pose)
 */
static Eigen::Isometry2d pose2isometry2d(Eigen::Quaterniond quat, Eigen::Vector3d pos) {
  double angle = quatToAngle(quat);
  Eigen::Rotation2D<double> rot(angle);

  Eigen::Isometry2d iso = Eigen::Isometry2d::Identity();
  iso.linear() = rot.toRotationMatrix();
  iso.translation() = pos.block<2,1>(0,0);
  return iso;
}

/**
 * @brief given an Eigen::Matrix3d (3D orientation) and a Eigen::Vector3d (3D position) return the corresponding Eigen::Isometry2d (2D pose)
 */
static Eigen::Isometry2d pose2isometry2d(Eigen::Matrix3d rot_mat, Eigen::Vector3d pos) {
  double angle = rotToAngle(rot_mat);
  Eigen::Rotation2D<double> rot(angle);

  Eigen::Isometry2d iso = Eigen::Isometry2d::Identity();
  iso.linear() = rot.toRotationMatrix();
  iso.translation() = pos.block<2,1>(0,0);
  return iso;
}
/**
 * @brief convert nav_msgs::Odometry to Eigen::Isometry2d (2D pose)
 */
static Eigen::Isometry2d odom2isometry2d(const nav_msgs::OdometryConstPtr& odom_msg) {
  const auto& orientation = odom_msg->pose.pose.orientation;
  const auto& position = odom_msg->pose.pose.position;

  Eigen::Quaterniond quat;
  quat.w() = orientation.w;
  quat.x() = orientation.x;
  quat.y() = orientation.y;
  quat.z() = orientation.z;

  Eigen::Vector3d pos(position.x, position.y, position.z);

  return pose2isometry2d(quat, pos);
}

/**
 * @brief convert Eigen::Isometry2d (2d pose) to Eigen::Isometry3d (3d pose)
 */
static Eigen::Isometry3d isometry2dto3d(Eigen::Isometry2d iso2d) {
  Eigen::Isometry3d iso3d = Eigen::Isometry3d::Identity();
  iso3d.linear().block<2,2>(0,0) = iso2d.linear();
  iso3d.translation().block<2,1>(0,0) = iso2d.translation();
  return iso3d;
}
/**
 * @brief convert Eigen::Isometry3d (3d pose) to Eigen::Isometry2d (2d pose)
 */
static Eigen::Isometry2d isometry3dto2d(Eigen::Isometry3d iso3d) {
  Eigen::Isometry2d iso2d = pose2isometry2d(iso3d.linear(), iso3d.translation());
  return iso2d;
}  
/**
 * @brief convert Eigen::Matrix4d (3d pose) to Eigen:Matrix3d (2d pose)
 */
static Eigen::Matrix3d matrix4dto3d(Eigen::Matrix4d m4d) {
  Eigen::Isometry2d iso2d = pose2isometry2d(m4d.block<3,3>(0,0), m4d.block<3,1>(0,3));
  return iso2d.matrix();
} 
/**
 * @brief convert Eigen::Matrix3d (2d pose) to Eigen::Matrix4d (3d pose)
 */
static Eigen::Matrix4d matrix3dto4d(Eigen::Matrix3d m3d) {
  Eigen::Matrix4d m4d = Eigen::Matrix4d::Identity();
  m4d.block<2,2>(0,0) = m3d.block<2,2>(0,0);
  m4d.block<2,1>(0,3) = m3d.block<2,1>(0,2);
  return m4d;
} 
/**
 * @brief convert Eigen::Matrix4f (3d float pose) to Eigen::Matrix3f (2d float pose)
 */
static Eigen::Matrix3f matrix4fto3f(Eigen::Matrix4f m4f) {
  Eigen::Isometry2d iso2d = pose2isometry2d(m4f.block<3,3>(0,0).cast<double>(), m4f.block<3,1>(0,3).cast<double>());
  return iso2d.matrix().cast<float>();
}
/**
 * @brief convert Eigen::Matrix4d (3d pose) to Eigen::Matrix4d (2d pose)
 */
static Eigen::Matrix4d state6to3(Eigen::Matrix4d in) {
  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  Eigen::Isometry2d temp = pose2isometry2d(in.block<3,3>(0,0), in.block<3,1>(0,3));
  out.block<2,2>(0,0) = temp.linear();
  out.block<2,1>(0,3) = temp.translation();
  return out;
}

}  // namespace hdl_graph_slam

#endif  // ROS_UTILS_HPP
