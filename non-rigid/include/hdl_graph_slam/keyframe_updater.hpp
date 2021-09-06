// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_UPDATER_HPP
#define KEYFRAME_UPDATER_HPP

#include <ros/ros.h>
#include <Eigen/Dense>

namespace hdl_graph_slam {

/**
 * @brief this class decides if a new frame should be registered to the pose graph as a keyframe
 */
class KeyframeUpdater {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief constructor
   * @param pnh
   */
  KeyframeUpdater(ros::NodeHandle& pnh) : is_first(true), prev_keypose(Eigen::Isometry2d::Identity()) {
    keyframe_delta_trans = pnh.param<double>("keyframe_delta_trans", 2.0);
    keyframe_delta_angle = pnh.param<double>("keyframe_delta_angle", 2.0);

    accum_distance = 0.0;
  }

  /**
   * @brief decide if a new frame should be registered to the graph
   * @param pose  pose of the frame
   * @return  if true, the frame should be registered
   */
  bool update(const Eigen::Isometry2d& pose) {
    // first frame is always registered to the graph
    if(is_first) {
      is_first = false;
      prev_keypose = pose;
      return true;
    }

    // calculate the delta transformation from the previous keyframe
    Eigen::Isometry2d delta = prev_keypose.inverse() * pose;
    double dx = delta.translation().norm();
    double da = Eigen::Rotation2D<double>(delta.linear()).angle();
    //std::cout << "prev_keypose: " << prev_keypose.matrix() << std::endl;
    //std::cout << "pose: " << pose.matrix() << std::endl;
    //std::cout << "delta: " << delta.matrix() << std::endl;

    //std::cout << "dx: " << dx << " thresh: " << keyframe_delta_trans << std::endl;
    //std::cout << "da: " << da << " thresh: " << keyframe_delta_angle << std::endl;

    // too close to the previous frame
    if(dx < keyframe_delta_trans && da < keyframe_delta_angle) {
      return false;
    }

    accum_distance += dx;
    prev_keypose = pose;
    return true;
  }

  /**
   * @brief the last keyframe's accumulated distance from the first keyframe
   * @return accumulated distance
   */
  double get_accum_distance() const {
    return accum_distance;
  }

private:
  // parameters
  double keyframe_delta_trans;  //
  double keyframe_delta_angle;  //

  bool is_first;
  double accum_distance;
  Eigen::Isometry2d prev_keypose;
};

}  // namespace hdl_graph_slam

#endif  // KEYFRAME_UPDATOR_HPP
