#ifndef BUILDING_HPP
#define BUILDING_HPP

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

namespace hdl_graph_slam {
	using PointT3 = pcl::PointXYZ;

	/**
    * @brief define a building object with the same structure as downloaded from openstreetmap
    */
	class Building {
		public: 
			std::string id; // id of the building
			std::map<std::string,std::string> tags; // tags associated to the building
			pcl::PointCloud<PointT3>::Ptr geometry; // geometry of the building already interpolated and referred to zero utm
			pcl::PointCloud<PointT3>::Ptr vertices; // just store vertices not interpolated - unused, may be removed
			Building(void); // empty building constructor
	};
}
#endif