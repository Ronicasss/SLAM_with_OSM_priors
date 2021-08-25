#include "hdl_graph_slam/building_tools.hpp"

//DEBUG = 1 -> print debug lines
//DEBUG different from 1 -> print nothing
#define DEBUG 0

namespace hdl_graph_slam {

	BuildingTools::BuildingTools(void) {
	}

	std::vector<Building> BuildingTools::parseBuildings(std::string result, Eigen::Vector3d zero_utm) {
		std::stringstream ss(result);
    	boost::property_tree::ptree pt;
		read_xml(ss, pt); // parse the result string as xml using boost
	 	std::vector< Node> nodes;
	 	std::vector<Building> b;
	 	try {
	 		// iterate over the results from parsing and read the children of the "osm" tag
	 		BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, pt.get_child("osm") ) {
				// a child may be a "node" (aka a geographic lla point) 
				//(all the nodes always come before all the ways in the xml) 
		        if( v.first == "node" ) {
		        	// build the struct Node
		        	 Node n;
		        	n.id = v.second.get<std::string>("<xmlattr>.id");
		        	n.lat = v.second.get<double>("<xmlattr>.lat");
		        	n.lon = v.second.get<double>("<xmlattr>.lon");
		            //std::cout << "node id: " << n.id << " lat: " << std::to_string(n.lat) << " lon: " << std::to_string(n.lat) << std::endl;
		            nodes.push_back(n);
		        }
		        if(v.first == "way") {
		        	// this child is a "way" (aka a building)
		        	Building btemp; // initialize an empty Building object that will be filled later
		        	std::vector<std::string> nd_refs;
		        	std::map<std::string,std::string> tags;
		        	
		        	// read and initialize the id of the building
		        	std::string way_id = v.second.get<std::string>("<xmlattr>.id");
		        	btemp.id = way_id;
		        	
					// iterate over the children of the "way" tag
					for(boost::property_tree::ptree::const_iterator v1 = v.second.begin(); v1 != v.second.end(); ++v1) {
						// this child is a "nd" tag (aka the ids of the nodes that constitute the way)
						if(v1->first == "nd") {
							std::string nd_ref = v1->second.get<std::string>("<xmlattr>.ref");
							nd_refs.push_back(nd_ref);
		        		}
						if(v1->first == "tag") {
							// this child is a "tag" tag (aka chracteristics of the way as key - value)
							std::string key = v1->second.get<std::string>("<xmlattr>.k");
							std::string value = v1->second.get<std::string>("<xmlattr>.v");
							tags[key] = value;
		        		}
					}
					btemp.tags = tags; // assign tags to the building
					
					// create the pointcloud associated to the building
					pcl::PointCloud<PointT3>::Ptr pc_temp(new pcl::PointCloud<PointT3>);
					*pc_temp = *(buildPointCloud(nd_refs,nodes,zero_utm));
					btemp.geometry = pc_temp; // assign the pointcloud to the building
					btemp.vertices = getVertices(nd_refs,nodes,zero_utm); // assign the vertices to the buildings
					b.push_back(btemp);
		        }
	    	}
	 	}
	 	catch(boost::property_tree::ptree_error & e){
		    std::cerr<< "No xml! error:" << e.what() << std::endl;
		}
    	return b;
	}

	pcl::PointCloud<PointT3>::Ptr BuildingTools::getVertices(std::vector<std::string> nd_refs, std::vector< Node> nodes, Eigen::Vector3d zero_utm) {
		pcl::PointCloud<PointT3>::Ptr pc_temp(new pcl::PointCloud<PointT3>);
		// iterate vertices references
		for(std::vector<std::string>::const_iterator it = nd_refs.begin(); it != nd_refs.end(); ++it) {
			Node n = getNode(*it, nodes); // get the node corresponding to current nd_ref
			PointT3 pt_temp = toUtm(Eigen::Vector3d(n.lat, n.lon, 0), zero_utm); // convert the node to utm and refer it to zero_utm
			pc_temp->push_back(pt_temp);
		}
		return pc_temp;		
	}

	pcl::PointCloud<PointT3>::Ptr BuildingTools::buildPointCloud(std::vector<std::string> nd_refs, std::vector< Node> nodes, Eigen::Vector3d zero_utm) {
		pcl::PointCloud<PointT3>::Ptr pc_temp(new pcl::PointCloud<PointT3>);
		Eigen::Vector3d previous;
		int first = 1;
		// iterate over vertices references
		for(std::vector<std::string>::const_iterator it = nd_refs.begin(); it != nd_refs.end(); ++it) {
			Node n = getNode(*it, nodes); // get the node corresponding to current nd_ref
			Eigen::Vector3d pt_temp; // create lla point
			pt_temp(0) = n.lat;
			pt_temp(1) = n.lon;
			pt_temp(2) = 0;
			
			if(first) {
				first = 0;
				previous = pt_temp;
			} else {
				// interpolate between two consecutive lla points and add it to the temporary pointcloud
				*pc_temp += *(interpolate(toUtm(previous, zero_utm), toUtm(pt_temp, zero_utm)));
				previous = pt_temp;
			}
		}
		return pc_temp;
	}

	BuildingTools::Node BuildingTools::getNode(std::string nd_ref, std::vector< Node> nodes) {
	 	// get the Node corresponding to the nd_ref
	 	Node n_temp;
		n_temp.id = "";
		n_temp.lat = 0;
		n_temp.lon = 0;
		// iterate over the nodes 
		for(std::vector< Node>::const_iterator it = nodes.begin(); it != nodes.end(); ++it) {
			if(nd_ref.compare(it->id) == 0) {
				n_temp = *it;
				break;
			}
		}
		return n_temp;
	}

	pcl::PointCloud<PointT3>::Ptr BuildingTools::interpolate(PointT3 a, PointT3 b) {
		// linear interpolation between a and b (both x and y)
		// return pts = a + t(b-a) if a < b, 
		// pts = a - t(a-b) if a > b, 
		// pts = a if a==b 
		float t = 0.001; // interpolation step
		int neg_x = 0, neg_y = 0;
		pcl::PointCloud<PointT3>::Ptr pc_temp(new pcl::PointCloud<PointT3>);
		if(a.x<0 && b.x<0) {
			neg_x = 1;
			a.x = -a.x;
			b.x = -b.x;
		}
		if(a.y<0 && b.y<0) {
			neg_y = 1;
			a.y = -a.y;
			b.y = -b.y;
		}

		for(float i=0;i<=1;i=i+t) {
			PointT3 pt;
		
			if(a.x == b.x) {
				pt.x = a.x;
			}
			else {
				if(a.x < b.x) {
					pt.x = a.x + (i*(b.x - a.x));
				} else {
					pt.x = a.x - (i*(a.x - b.x));
				}
			}
			
			if(a.y == b.y) {
				pt.y = a.y;
			}
			else {
				if(a.y < b.y) {
					pt.y = a.y + (i*(b.y - a.y));
				} else {
					pt.y = a.y - (i*(a.y - b.y));
				}
			}
			pt.z = 0;
			if(neg_x)
				pt.x = -pt.x;
			if(neg_y)
				pt.y = -pt.y;
			pc_temp->push_back(pt);
			if(a.x == b.x && a.y == b.y)
				break;
		}
		return pc_temp;
	}

	std::string BuildingTools::downloadBuildings(double lat, double lon, double rad, std::string host) {
		std::string result;
		try {
			// try to do the request to the host to download buildings
			std::string url = host + "/api/interpreter?data=way[%27building%27](around:" + std::to_string(rad) + "," + std::to_string(lat) + "," + std::to_string(lon) + ");%20(._;%3E;);out;";
	        std::cout << url << std::endl;
	        curlpp::Cleanup cleaner;
	        curlpp::Easy request;

	        // Setting the URL to retrive.
	        std::ostringstream os;
	        curlpp::options::WriteStream ws(&os);

	        
	        request.setOpt(new curlpp::options::Url(url));
	        request.setOpt(ws);
	        request.perform();

	        result = os.str();
      	}
		catch ( curlpp::LogicError & e ) {
			std::cout << "curlpp logic error: " << e.what() << std::endl;
		}
		catch ( curlpp::RuntimeError & e ) {
			std::cout << "curlpp runtime error: " << e.what() << std::endl;
		}
		return result;
	}

	// toUtm converts to utm and already refer to zero_utm
	PointT3 BuildingTools::toUtm(Eigen::Vector3d pt, Eigen::Vector3d zero_utm) {
		geographic_msgs::GeoPoint pt_lla;
    	geodesy::UTMPoint pt_utm;

    	pt_lla.latitude = pt(0);
    	pt_lla.longitude = pt(1);
    	pt_lla.altitude = 0;
    	geodesy::fromMsg(pt_lla, pt_utm); 
    	
    	return PointT3((pt_utm.easting-zero_utm(0)), (pt_utm.northing-zero_utm(1)), 0);
	}

	std::vector<Building> BuildingTools::getBuildings(double lat, double lon, double rad, Eigen::Vector2d zero_utm, std::string host){
		std::string result = downloadBuildings(lat, lon, rad, host);
		
		Eigen::Vector3d v;
		v.block<2,1>(0,0) = zero_utm;
		v(2) = 0;
		std::vector<Building> tmp = parseBuildings(result,v);
		return tmp;
	}
}