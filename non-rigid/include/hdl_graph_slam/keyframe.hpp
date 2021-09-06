// SPDX-License-Identifier: BSD-2-Clause

#ifndef KEYFRAME_HPP
#define KEYFRAME_HPP

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/optional.hpp>
#include "hdl_graph_slam/building_node.hpp"

namespace g2o {
class VertexSE2;
class HyperGraph;
class SparseOptimizer;
}  // namespace g2o

namespace hdl_graph_slam {

/**
 * @brief KeyFrame (pose node)
 */
struct KeyFrame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PointT = pcl::PointXYZI;
  using Ptr = std::shared_ptr<KeyFrame>;

  KeyFrame(const ros::Time& stamp, const Eigen::Isometry2d& odom, double accum_distance, const pcl::PointCloud<PointT>::ConstPtr& cloud, int index);
  KeyFrame(const std::string& directory, g2o::HyperGraph* graph);
  virtual ~KeyFrame();

  void save(const std::string& directory);
  bool load(const std::string& directory, g2o::HyperGraph* graph);

  long id() const;
  Eigen::Isometry2d estimate() const;

public:
  ros::Time stamp;                                // timestamp
  Eigen::Isometry2d odom;                         // odometry (estimated by scan_matching_odometry)
  double accum_distance;                          // accumulated distance from the first node (by scan_matching_odometry)
  pcl::PointCloud<PointT>::ConstPtr cloud;        // point cloud
  boost::optional<Eigen::Vector2d> utm_coord;     // UTM coord obtained by GPS

  boost::optional<Eigen::Vector2d> acceleration;    //
  boost::optional<Eigen::Rotation2D<double>> orientation;  //

  g2o::VertexSE2* node;  // node instance

  int index;
  boost::optional<int> utm_zone;                  // UTM zone and band needed for conversion to lla
  boost::optional<char> utm_band; 
  std::vector<BuildingNode::Ptr> buildings_nodes;       // buildings associated to this keyframe
  bool buildings_check;
  pcl::PointCloud<pcl::PointXYZ>::Ptr odomCloud;      
};

/**
 * @brief KeyFramesnapshot for map cloud generation
 */
struct KeyFrameSnapshot {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using PointT = KeyFrame::PointT;
  using Ptr = std::shared_ptr<KeyFrameSnapshot>;

  KeyFrameSnapshot(const KeyFrame::Ptr& key);
  KeyFrameSnapshot(const Eigen::Isometry2d& pose, const pcl::PointCloud<PointT>::ConstPtr& cloud);

  ~KeyFrameSnapshot();

public:
  Eigen::Isometry2d pose;                   // pose estimated by graph optimization
  pcl::PointCloud<PointT>::ConstPtr cloud;  // point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr odomCloud;
};

}  // namespace hdl_graph_slam

#endif  // KEYFRAME_HPP
