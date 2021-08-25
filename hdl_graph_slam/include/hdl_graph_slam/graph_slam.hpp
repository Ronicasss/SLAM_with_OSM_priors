// SPDX-License-Identifier: BSD-2-Clause

#ifndef GRAPH_SLAM_HPP
#define GRAPH_SLAM_HPP

#include <memory>
#include <ros/time.h>

#include <g2o/core/hyper_graph.h>

namespace g2o {
class VertexSE2;
class EdgeSE2PriorXY;
class EdgeSE2PriorQuat;
class EdgeSE2Prior;
class EdgeSE2;
class VertexPointXY;
class EdgeSE2PointXY;
class EdgeSE2XYPrior;
class EdgeSE2PointXYCustom;
class EdgeXYPrior;
class RobustKernelFactory;
}  // namespace g2o

namespace hdl_graph_slam {

class GraphSLAM {
public:
  GraphSLAM(const std::string& solver_type = "lm_var");
  virtual ~GraphSLAM();

  int num_vertices() const;
  int num_edges() const;

  void set_solver(const std::string& solver_type);
  /**
   * @brief add a 2D position prior custom edge to a SE2 node
   */
  g2o::EdgeSE2PriorXY* add_se2_prior_xy_edge(g2o::VertexSE2* v_se2, const Eigen::Vector2d& xy, const Eigen::MatrixXd& information_matrix);
  /**
   * @brief add a 2D orientation prior custom edge to a SE2 node
   */
  g2o::EdgeSE2PriorQuat* add_se2_prior_quat_edge(g2o::VertexSE2* v_se2, const Eigen::Rotation2D<double>& quat, const Eigen::MatrixXd& information_matrix);
  /**
   * @brief add a 2D pose prior edge to a SE2 node
   */
  g2o::EdgeSE2Prior* add_se2_edge_prior(g2o::VertexSE2* v1, const Eigen::Isometry2d& relative_pose, const Eigen::MatrixXd& information_matrix);
  /**
   * @brief add a SE2 node
   */
  g2o::VertexSE2* add_se2_node(const Eigen::Isometry2d& pose);
  /**
   * @brief add a 2D pose edge between two SE2 nodes
   */
  g2o::EdgeSE2* add_se2_edge(g2o::VertexSE2* v1, g2o::VertexSE2* v2, const Eigen::Isometry2d& relative_pose, const Eigen::MatrixXd& information_matrix);
  /**
   * @brief add a 2D position custom edge between a SE2 node and a PointXY node
   */
  g2o::EdgeSE2PointXYCustom* add_se2_pointxy_edge(g2o::VertexSE2* v1, g2o::VertexPointXY* v2, const Eigen::Vector2d& relative_pose, const Eigen::MatrixXd& information_matrix);
  /**
   * @brief add a PointXY node
   */
  g2o::VertexPointXY* add_pointxy_node(const Eigen::Vector2d& pose);
  /**
   * @brief add a 2D position prior edge to a SE2 node
   */
  g2o::EdgeSE2XYPrior* add_se2_pointxy_prior(g2o::VertexSE2* v1, const Eigen::Vector2d& relative_pose, const Eigen::MatrixXd& information_matrix);
  /**
   * @brief add a 2D position prior edge to a PointXY node
   */
  g2o::EdgeXYPrior* add_pointxy_prior(g2o::VertexPointXY* v, const Eigen::Vector2d& pose, const Eigen::MatrixXd& information_matrix);

  void add_robust_kernel(g2o::HyperGraph::Edge* edge, const std::string& kernel_type, double kernel_size);

  /**
   * @brief perform graph optimization
   */
  int optimize(int num_iterations);

  /**
   * @brief save the pose graph to a file
   * @param filename  output filename
   */
  void save(const std::string& filename);

  /**
   * @brief load the pose graph from file
   * @param filename  output filename
   */
  bool load(const std::string& filename);

public:
  g2o::RobustKernelFactory* robust_kernel_factory;
  std::unique_ptr<g2o::HyperGraph> graph;  // g2o graph
};

}  // namespace hdl_graph_slam

#endif  // GRAPH_SLAM_HPP
