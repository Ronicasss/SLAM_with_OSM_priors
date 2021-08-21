#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include "hdl_graph_slam/building.hpp"
#include <fstream>
#include <regex>
#include <string>
#include <cmath>
#include <sstream>
#include <cstdlib>
#include <iomanip>

#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Exception.hpp>

namespace hdl_graph_slam {
	/**
   * @brief class with static methods to download and parse buildings
   */
	class BuildingTools {
		public: 
			static std::vector<Building> getBuildings(double lat, double lon, double rad, Eigen::Vector2d zero_utm, std::string host);
			
			/**
			   * @brief struct Node define an openstreetmap node
			   */
			struct Node
			{
			    std::string id;
			    double lat;
			    double lon;
			};
			BuildingTools(void);
		private:
			/**
			 * @brief download buildings 
			 * @param latitude
			 * @param longitude
			 * @param radius centered in (lat, lon)
			 * @param host server to made the download request
			 * @return the string containing the response from the host
			 */
			static std::string downloadBuildings(double lat, double lon, double rad, std::string host);
			/**
			 * @brief parse the buildings from a string to Building objects
			 * @param string response from host
			 * @param zero_utm 
			 * @return vector of Buildings
			 */
			static std::vector<Building> parseBuildings(std::string result, Eigen::Vector3d zero_utm);
			/**
			 * @brief create a pointcloud of a building based on the references given in input (already interpolated and referred to zero_utm)
			 * @param vector of nd_ref that represent the ids of osm nodes that are vertices of the building
			 * @param list of all osm nodes from the response
			 * @param zero_utm
			 * @return pointcloud representing the outline of a building
			 */
			static pcl::PointCloud<PointT3>::Ptr buildPointCloud(std::vector<std::string> nd_refs, std::vector<Node> nodes,Eigen::Vector3d zero_utm);
			/**
			 * @brief create a pointcloud containing just the vertices of the building (already referred to zero_utm)
			 * @param vector of nd_ref that represent the ids of osm nodes that are vertices of the building
			 * @param list of all osm nodes from the response
			 * @param zero_utm
			 * @return pointcloud representing the vertices of a building
			 */
			static pcl::PointCloud<PointT3>::Ptr getVertices(std::vector<std::string> nd_refs, std::vector<Node> nodes,Eigen::Vector3d zero_utm);
			/**
			 * @brief retrieve an osm node given its id
			 * @param id of the wanted node
			 * @param list of all osm nodes from the response
			 * @return osm node
			 */
			static Node getNode(std::string nd_ref, std::vector<Node> nodes);
			/**
			 * @brief convert a lla point to utm and refers it to zero_utm
			 * @param lla point to convert
			 * @param zero_utm
			 * @return point converted to utm and referred to zero_utm 
			 */
			static PointT3 toUtm(Eigen::Vector3d pt, Eigen::Vector3d zero_utm);
			/**
			 * @brief linear interpolation between two points
			 * @param starting point
			 * @param ending point
			 * @return pointcloud containing all the interpolated points between a and b
			 */
			static pcl::PointCloud<PointT3>::Ptr interpolate(PointT3 a, PointT3 b);	
	};
}