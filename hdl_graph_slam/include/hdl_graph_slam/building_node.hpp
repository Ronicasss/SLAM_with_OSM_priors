#ifndef BUILDING_NODE
#define BUILDING_NODE

#include <ros/ros.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include "hdl_graph_slam/building.hpp"

namespace g2o {
  class VertexSE2;
}

namespace hdl_graph_slam {
	/**
   * @brief define a building as an entity in the graph
   */
	class BuildingNode {
		private:
			void setOrigin(); // set local_origin						
		public: 
			typedef boost::shared_ptr<BuildingNode> Ptr; // define the pointer to a BuildingNode
			Building building;	// building to which the node refers
			pcl::PointCloud<PointT3>::Ptr referenceSystem;	// pc containing all building points referred to local_origin (from building.geometry) 
			Eigen::Vector2d local_origin; // south-westernmost point of the building wrt zero_utm
			g2o::VertexSE2* node; // g2o node representing the building 

			BuildingNode(); // constructor
			void setReferenceSystem(); // set referenceSystem
			pcl::PointCloud<PointT3>::Ptr setVerticesReferenceSystem(); // refers building.vertices to the local origin and return it
	};
}
#endif