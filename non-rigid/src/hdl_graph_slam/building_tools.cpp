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
		read_xml(ss, pt);
	 	std::vector< Node> nodes;
	 	std::vector<Building> b;
	 	try {
	 		BOOST_FOREACH( boost::property_tree::ptree::value_type const& v, pt.get_child("osm") ) {
				//std::cout << "first: " << v.first << std::endl;
		        if( v.first == "node" ) {
		        	 Node n;
		        	n.id = v.second.get<std::string>("<xmlattr>.id");
		        	n.lat = v.second.get<double>("<xmlattr>.lat");
		        	n.lon = v.second.get<double>("<xmlattr>.lon");
		            //std::cout << "node id: " << n.id << " lat: " << std::to_string(n.lat) << " lon: " << std::to_string(n.lat) << std::endl;
		            nodes.push_back(n);
		        }
		        if(v.first == "way") {
		        	Building btemp;
		        	std::vector<std::string> nd_refs;
		        	std::map<std::string,std::string> tags;
		        	std::string way_id = v.second.get<std::string>("<xmlattr>.id");
		        	btemp.id = way_id;
		        	//std::cout << "way id: " << way_id << std::endl;

					for(boost::property_tree::ptree::const_iterator v1 = v.second.begin(); v1 != v.second.end(); ++v1) {
						//std::cout << "way first: " << v1->first << std::endl;
						if(v1->first == "nd") {
							std::string nd_ref = v1->second.get<std::string>("<xmlattr>.ref");
							nd_refs.push_back(nd_ref);
		        			//std::cout << "nd ref: " << nd_ref << std::endl;
						}
						if(v1->first == "tag") {
							std::string key = v1->second.get<std::string>("<xmlattr>.k");
							std::string value = v1->second.get<std::string>("<xmlattr>.v");
							tags[key] = value;
		        			//std::cout << "k: " << key << " v: " << value << std::endl;
						}
					}
					btemp.tags = tags;
					pcl::PointCloud<PointT3>::Ptr pc_temp(new pcl::PointCloud<PointT3>);
					*pc_temp = *(buildPointCloud(nd_refs,nodes,zero_utm));
					btemp.geometry = pc_temp;
					btemp.vertices = getVertices(nd_refs,nodes,zero_utm);
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
		for(std::vector<std::string>::const_iterator it = nd_refs.begin(); it != nd_refs.end(); ++it) {
			Node n = getNode(*it, nodes);
			PointT3 pt_temp = toUtm(Eigen::Vector3d(n.lat, n.lon, 0), zero_utm);
			pc_temp->push_back(pt_temp);
		}
		return pc_temp;		
	}

	pcl::PointCloud<PointT3>::Ptr BuildingTools::buildPointCloud(std::vector<std::string> nd_refs, std::vector< Node> nodes, Eigen::Vector3d zero_utm) {
		pcl::PointCloud<PointT3>::Ptr pc_temp(new pcl::PointCloud<PointT3>);
		Eigen::Vector3d previous;
		int first = 1;
		for(std::vector<std::string>::const_iterator it = nd_refs.begin(); it != nd_refs.end(); ++it) {
			Node n = getNode(*it, nodes);
			Eigen::Vector3d pt_temp;
			pt_temp(0) = n.lat;
			pt_temp(1) = n.lon;
			pt_temp(2) = 0;
			//std::cout << "pt_temp: " << pt_temp(0) << "," << pt_temp(1) << std::endl;
			if(first) {
				first = 0;
				previous = pt_temp;
			} else {
				*pc_temp += *(interpolate(toUtm(previous, zero_utm), toUtm(pt_temp, zero_utm)));
				previous = pt_temp;
			}
			//std::cout << "previous: " << previous(0) << "," << previous(1) << std::endl;

		}
		//std::cout << "build point cloud size: " << pc_temp->size() << std::endl;
		return pc_temp;
	}

	BuildingTools::Node BuildingTools::getNode(std::string nd_ref, std::vector< Node> nodes) {
	 	Node n_temp;
		n_temp.id = "";
		n_temp.lat = 0;
		n_temp.lon = 0;
		for(std::vector< Node>::const_iterator it = nodes.begin(); it != nodes.end(); ++it) {
			if(nd_ref.compare(it->id) == 0) {
				n_temp = *it;
				break;
			}
		}
		//std::cout << "get node: " << n_temp.id << std::endl;
		return n_temp;
	}

	pcl::PointCloud<PointT3>::Ptr BuildingTools::interpolate(PointT3 a, PointT3 b) {
		// linear interpolation between a and b (both x and y)
		// return pts = a + t(b-a) if a < b, 
		// pts = a - t(a-b) if a > b, 
		// pts = a if a==b 
		//std::cout << "a.x: " << a.x << " a.y: " << a.y << std::endl; 
		//std::cout << "b.x: " << b.x << " b.y: " << b.y << std::endl; 
		float t = 0.001;
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
					//std::cout << typeid(pt.x).name() << std::endl;
					//std::cout << "prova: " << (float)(a.x - (i*(a.x - b.x))) << std::endl;
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
			//std::cout << "i: " << i << std::endl;
			//std::cout << "int x: " << pt.x << ", int y: " << pt.y << std::endl;
			pc_temp->push_back(pt);
			if(a.x == b.x && a.y == b.y)
				break;
		}
		return pc_temp;
	}

	std::string BuildingTools::downloadBuildings(double lat, double lon, double rad, std::string host) {
		std::string result;
		try {
			//std::string url = "https://overpass.openstreetmap.ru/api/interpreter?data=way[%27building%27](around:" + std::to_string(rad) + "," + std::to_string(lat) + "," + std::to_string(lon) + ");%20(._;%3E;);out;";
			
			//std::string url = "https://overpass.nchc.org.tw/api/interpreter?data=way[%27building%27](around:" + std::to_string(rad) + "," + std::to_string(lat) + "," + std::to_string(lon) + ");%20(._;%3E;);out;";
			
			std::string url = host + "/api/interpreter?data=way[%27building%27](around:" + std::to_string(rad) + "," + std::to_string(lat) + "," + std::to_string(lon) + ");%20(._;%3E;);out;";
	        //std::string url = "https://fleet.ls.hereapi.com/1/search/proximity.json?key_attributes=BUILDING_ID&proximity=" + std::to_string(lat) + "," + std::to_string(lon) + "," + std::to_string(rad) + "&layer_ids=BUILDING&apiKey=C0LqxwuV0lzyiXA6WKEB84h8f8AHK4EMLum1cGGIGI8";
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
    	 //std::cout << "toutm zone: " << std::to_string(pt_utm.zone) << std::endl;
    	  //std::cout << "toutm band: " << pt_utm.band << std::endl;
    	return PointT3((pt_utm.easting-zero_utm(0)), (pt_utm.northing-zero_utm(1)), 0);
	}

	std::vector<Building> BuildingTools::getBuildings(double lat, double lon, double rad, Eigen::Vector2d zero_utm, std::string host){
		std::string result = downloadBuildings(lat, lon, rad, host);
		//std::cout << "Result: " << result << std::endl;
		Eigen::Vector3d v;
		v.block<2,1>(0,0) = zero_utm;
		v(2) = 0;
		std::vector<Building> tmp = parseBuildings(result,v);
		return tmp;
	}
}