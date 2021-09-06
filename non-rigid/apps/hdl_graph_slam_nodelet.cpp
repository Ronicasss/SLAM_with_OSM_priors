// SPDX-License-Identifier: BSD-2-Clause

#include <ctime>
#include <mutex>
#include <atomic>
#include <memory>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Sentence.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <hdl_graph_slam/SaveMap.h>
#include <hdl_graph_slam/DumpGraph.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <hdl_graph_slam/ros_utils.hpp>
#include <hdl_graph_slam/ros_time_hash.hpp>

#include <hdl_graph_slam/graph_slam.hpp>
#include <hdl_graph_slam/keyframe.hpp>
#include <hdl_graph_slam/keyframe_updater.hpp>
#include <hdl_graph_slam/loop_detector.hpp>
#include <hdl_graph_slam/information_matrix_calculator.hpp>
#include <hdl_graph_slam/map_cloud_generator.hpp>
#include <hdl_graph_slam/nmea_sentence_parser.hpp>

#include <g2o/edge_se2_priorxy.hpp>
#include <g2o/edge_se2_priorquat.hpp>
#include <g2o/edge_se2_pointxy_custom.hpp>

//includes for rigid slam
#include "hdl_graph_slam/building_tools.hpp"
#include "hdl_graph_slam/building_node.hpp"
#include <pclomp/gicp_omp.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include <tf/transform_broadcaster.h>
// use g2o pre-defined se2 classes
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/vertex_se2.h>

#include <pclomp/ndt_omp.h>

namespace hdl_graph_slam {

class HdlGraphSlamNodelet : public nodelet::Nodelet {
public:
  typedef pcl::PointXYZI PointT;
  typedef pcl::PointXYZ PointT3;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> ApproxSyncPolicy;

  HdlGraphSlamNodelet() {}
  virtual ~HdlGraphSlamNodelet() {}

  virtual void onInit() {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    // init parameters
    map_frame_id = private_nh.param<std::string>("map_frame_id", "map");
    odom_frame_id = private_nh.param<std::string>("odom_frame_id", "odom");
    map_cloud_resolution = private_nh.param<double>("map_cloud_resolution", 0.05);
    trans_odom2map.setIdentity();

    max_keyframes_per_update = private_nh.param<int>("max_keyframes_per_update", 10);

    // buildings parameters
    buildings_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/buildings_cloud", 1);
    odom_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/odom_cloud", 32);
    transformed_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/transformed_cloud", 32);
    estimated_buildings_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/estimated_buildings", 32);
    original_odom_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/original_odom_cloud", 32);
    all_buildings_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/all_buildings", 32);
    temp_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/temp", 32);
    zero_utm_pub = mt_nh.advertise<sensor_msgs::NavSatFix>("/hdl_graph_slam/zero_utm", 32);

    lidar_range = private_nh.param<float>("lidar_range", 20);
    ground_floor_max_thresh = private_nh.param<double>("ground_floor_max_thresh", 0.5);
    radius_search = private_nh.param<double>("radius_search", 1);
    min_neighbors_in_radius = private_nh.param<double>("min_neighbors_in_radius", 100);
    enter = private_nh.param<bool>("enable_buildings", true);
    first_guess = true;
    prev_guess = Eigen::Matrix4f::Identity();
    fix_first_building = private_nh.param<bool>("fix_first_building", true);
    reset_gicp = false;
    reset_counter = 0;
    dist_last_kf = 0.0;
    //pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
    zero_utm_lla = Eigen::Vector3d::Zero();
    ii = -1;

    //read ground truth
    gt = loadPoses(private_nh.param<std::string>("gt_path", ""));
    std::cout << "gt poses loaded: " << gt.size() << std::endl;
    // publisher to visualize ground truth
    gt_markers_pub = mt_nh.advertise<visualization_msgs::Marker>("/hdl_graph_slam/gt_markers", 16);
    // end buildings parameters

    anchor_node = nullptr;
    anchor_edge = nullptr;
    graph_slam.reset(new GraphSLAM(private_nh.param<std::string>("g2o_solver_type", "lm_var")));
    keyframe_updater.reset(new KeyframeUpdater(private_nh));
    loop_detector.reset(new LoopDetector(private_nh));
    map_cloud_generator.reset(new MapCloudGenerator());
    inf_calclator.reset(new InformationMatrixCalculator(private_nh));
    nmea_parser.reset(new NmeaSentenceParser());

    gps_time_offset = private_nh.param<double>("gps_time_offset", 0.0);
    gps_edge_stddev_xy = private_nh.param<double>("gps_edge_stddev_xy", 10000.0);
    
    imu_time_offset = private_nh.param<double>("imu_time_offset", 0.0);
    enable_imu_orientation = private_nh.param<bool>("enable_imu_orientation", false);
    enable_imu_acceleration = private_nh.param<bool>("enable_imu_acceleration", false);
    imu_orientation_edge_stddev = private_nh.param<double>("imu_orientation_edge_stddev", 0.1);
    imu_acceleration_edge_stddev = private_nh.param<double>("imu_acceleration_edge_stddev", 3.0);

    points_topic = private_nh.param<std::string>("points_topic", "/velodyne_points");

    // subscribers
    odom_sub.reset(new message_filters::Subscriber<nav_msgs::Odometry>(mt_nh, "/odom", 256));
    cloud_sub.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(mt_nh, "/filtered_points", 32));
    sync.reset(new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(32), *odom_sub, *cloud_sub));
    sync->registerCallback(boost::bind(&HdlGraphSlamNodelet::cloud_callback, this, _1, _2));
    imu_sub = nh.subscribe("/gpsimu_driver/imu_data", 1024, &HdlGraphSlamNodelet::imu_callback, this);
    
    if(private_nh.param<bool>("enable_gps", true)) {
      gps_sub = mt_nh.subscribe("/gps/geopoint", 1024, &HdlGraphSlamNodelet::gps_callback, this);
      nmea_sub = mt_nh.subscribe("/gpsimu_driver/nmea_sentence", 1024, &HdlGraphSlamNodelet::nmea_callback, this);
      navsat_sub = mt_nh.subscribe("/gps/navsat", 1024, &HdlGraphSlamNodelet::navsat_callback, this);
    }

    // publishers
    markers_pub = mt_nh.advertise<visualization_msgs::MarkerArray>("/hdl_graph_slam/markers", 16);
    odom2map_pub = mt_nh.advertise<geometry_msgs::TransformStamped>("/hdl_graph_slam/odom2pub", 16);
    map_points_pub = mt_nh.advertise<sensor_msgs::PointCloud2>("/hdl_graph_slam/map_points", 1, true);
    read_until_pub = mt_nh.advertise<std_msgs::Header>("/hdl_graph_slam/read_until", 32);

    dump_service_server = mt_nh.advertiseService("/hdl_graph_slam/dump", &HdlGraphSlamNodelet::dump_service, this);
    save_map_service_server = mt_nh.advertiseService("/hdl_graph_slam/save_map", &HdlGraphSlamNodelet::save_map_service, this);

    graph_updated = false;
    double graph_update_interval = private_nh.param<double>("graph_update_interval", 3.0);
    double map_cloud_update_interval = private_nh.param<double>("map_cloud_update_interval", 10.0);
    optimization_timer = mt_nh.createWallTimer(ros::WallDuration(graph_update_interval), &HdlGraphSlamNodelet::optimization_timer_callback, this);
    map_publish_timer = mt_nh.createWallTimer(ros::WallDuration(map_cloud_update_interval), &HdlGraphSlamNodelet::map_points_publish_timer_callback, this);
  }

private:
  /**
   * @brief received point clouds are pushed to #keyframe_queue
   * @param odom_msg
   * @param cloud_msg
   */
  void cloud_callback(const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    // ii is used to keep track of the order of arrival of keyframes, that may be different from the
    // index of a keyframe in the "keyframes" array. It is used to correctly compare a keyframe with the
    // corresponding ground truth
    ii++;
    std::cout << "ii: " << ii << std::endl;
    std::cout << "gt size: " << gt.size() << std::endl;
    std::cout << "kf size: " << keyframes.size() << std::endl;

    const ros::Time& stamp = cloud_msg->header.stamp;
    Eigen::Isometry2d odom = odom2isometry2d(odom_msg);
    
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if(base_frame_id.empty()) {
      base_frame_id = cloud_msg->header.frame_id;
    }

    if(!keyframe_updater->update(odom)) {
      std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
      if(keyframe_queue.empty()) {
        std_msgs::Header read_until;
        read_until.stamp = stamp + ros::Duration(10, 0);
        read_until.frame_id = points_topic;
        read_until_pub.publish(read_until);
        read_until.frame_id = "/filtered_points";
        read_until_pub.publish(read_until);
      }
      return;
    }

    double accum_d = keyframe_updater->get_accum_distance();
    KeyFrame::Ptr keyframe(new KeyFrame(stamp, odom, accum_d, cloud, ii));

    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);
    keyframe_queue.push_back(keyframe);
  }

  /**
   * @brief this method adds all the keyframes in #keyframe_queue to the pose graph (odometry edges)
   * @return if true, at least one keyframe was added to the pose graph
   */
  bool flush_keyframe_queue() {
    std::lock_guard<std::mutex> lock(keyframe_queue_mutex);

    if(keyframe_queue.empty()) {
      return false;
    }

    trans_odom2map_mutex.lock();
    Eigen::Isometry2d odom2map(trans_odom2map.cast<double>());
    trans_odom2map_mutex.unlock();

    int num_processed = 0;
    for(int i = 0; i < std::min<int>(keyframe_queue.size(), max_keyframes_per_update); i++) {
      num_processed = i;

      const auto& keyframe = keyframe_queue[i];
      // new_keyframes will be tested later for loop closure
      new_keyframes.push_back(keyframe);

      // add pose node
      Eigen::Isometry2d odom = odom2map * keyframe->odom;
      keyframe->node = graph_slam->add_se2_node(odom);
      keyframe_hash[keyframe->stamp] = keyframe;

      // fix the first node
      if(keyframes.empty() && new_keyframes.size() == 1) {
        if(private_nh.param<bool>("fix_first_node", false)) {
          Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(3, 3);
          std::stringstream sst(private_nh.param<std::string>("fix_first_node_stddev", "1 1 1"));
          for(int i = 0; i < 3; i++) {
            double stddev = 1.0;
            sst >> stddev;
            inf(i, i) = 1.0 / stddev;
          }
          std::cout << "fixed first keyframe" << std::endl;
          anchor_node = graph_slam->add_se2_node(Eigen::Isometry2d::Identity());
          anchor_node->setFixed(true);
          anchor_edge = graph_slam->add_se2_edge(anchor_node, keyframe->node, Eigen::Isometry2d::Identity(), inf);
        }
      }

      if(i == 0 && keyframes.empty()) {
        continue;
      }

      // add edge between consecutive keyframes
      const auto& prev_keyframe = i == 0 ? keyframes.back() : keyframe_queue[i - 1];

      Eigen::Isometry2d relative_pose = keyframe->odom.inverse() * prev_keyframe->odom;
      Eigen::MatrixXd information = inf_calclator->calc_information_matrix(keyframe->cloud, prev_keyframe->cloud, isometry2dto3d(relative_pose));
      // information matrix computed as 3D and converted to 2D
      Eigen::Matrix3d inf_3d = Eigen::Matrix3d::Identity();
      inf_3d.block<2,2>(0,0) = information.block<2,2>(0,0);
      inf_3d(2,2) = information(5,5);
      auto edge = graph_slam->add_se2_edge(keyframe->node, prev_keyframe->node, relative_pose, inf_3d);
      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("odometry_edge_robust_kernel", "NONE"), private_nh.param<double>("odometry_edge_robust_kernel_size", 1.0));
    }

    std_msgs::Header read_until;
    read_until.stamp = keyframe_queue[num_processed]->stamp + ros::Duration(10, 0);
    read_until.frame_id = points_topic;
    read_until_pub.publish(read_until);
    read_until.frame_id = "/filtered_points";
    read_until_pub.publish(read_until);

    keyframe_queue.erase(keyframe_queue.begin(), keyframe_queue.begin() + num_processed + 1);
    return true;
  }

  void nmea_callback(const nmea_msgs::SentenceConstPtr& nmea_msg) {
    GPRMC grmc = nmea_parser->parse(nmea_msg->sentence);

    if(grmc.status != 'A') {
      return;
    }

    geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
    gps_msg->header = nmea_msg->header;
    gps_msg->position.latitude = grmc.latitude;
    gps_msg->position.longitude = grmc.longitude;
    gps_msg->position.altitude = NAN;

    gps_callback(gps_msg);
  }

  void navsat_callback(const sensor_msgs::NavSatFixConstPtr& navsat_msg) {
    geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
    gps_msg->header = navsat_msg->header;
    gps_msg->position.latitude = navsat_msg->latitude;
    gps_msg->position.longitude = navsat_msg->longitude;
    gps_msg->position.altitude = navsat_msg->altitude;
    gps_callback(gps_msg);
  }

  /**
   * @brief received gps data is added to #gps_queue
   * @param gps_msg
   */
  void gps_callback(const geographic_msgs::GeoPointStampedPtr& gps_msg) {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);
    gps_msg->header.stamp += ros::Duration(gps_time_offset);
    gps_queue.push_back(gps_msg);
  }

  /**
   * @brief
   * @return
   */
  bool flush_gps_queue() {
    std::lock_guard<std::mutex> lock(gps_queue_mutex);
    
    if(keyframes.empty() || gps_queue.empty()) {
      return false;
    }

    bool updated = false;
    auto gps_cursor = gps_queue.begin();

    for(auto& keyframe : keyframes) {
      if(keyframe->stamp > gps_queue.back()->header.stamp) {
        break;
      }

      if(keyframe->stamp < (*gps_cursor)->header.stamp || keyframe->utm_coord) {
        continue;
      }

      // find the gps data which is closest to the keyframe
      auto closest_gps = gps_cursor;
      for(auto gps = gps_cursor; gps != gps_queue.end(); gps++) {
        auto dt = ((*closest_gps)->header.stamp - keyframe->stamp).toSec();
        auto dt2 = ((*gps)->header.stamp - keyframe->stamp).toSec();
        if(std::abs(dt) < std::abs(dt2)) {
          break;
        }

        closest_gps = gps;
      }

      // if the time residual between the gps and keyframe is too large, skip it
      gps_cursor = closest_gps;
      if(0.2 < std::abs(((*closest_gps)->header.stamp - keyframe->stamp).toSec())) {
        continue;
      }

      // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
      geodesy::UTMPoint utm;
      geodesy::fromMsg((*closest_gps)->position, utm);
      Eigen::Vector2d xyz(utm.easting, utm.northing);

      // the first gps data position will be the origin of the map
      if(!zero_utm) {
        zero_utm = xyz;
        // to convert from utm to lla (needed to download the buildings later), I need to store
        // also the utm zone and band, since the geodesy package requires them to perform the conversion.
        zero_utm_zone = utm.zone;
        zero_utm_band = utm.band;

        // this publisher is needed for the rviz satellite package:
        // it publishes the zero_utm in lla coords
        sensor_msgs::NavSatFix tmp;
        tmp.header.frame_id = "map";
        tmp.header.stamp = keyframe->stamp;
        zero_utm_lla(0) = (*closest_gps)->position.latitude;
        zero_utm_lla(1) = (*closest_gps)->position.longitude;
        zero_utm_lla(2) = (*closest_gps)->position.altitude;
        tmp.latitude = zero_utm_lla(0);
        tmp.longitude = zero_utm_lla(1);
        tmp.altitude = zero_utm_lla(2);

        zero_utm_pub.publish(tmp);
      }

      xyz -= (*zero_utm);

      keyframe->utm_coord = xyz;
      // store utm zone and band for conversion to lla to download buildings
      keyframe->utm_zone = utm.zone;
      keyframe->utm_band = utm.band;

      if(private_nh.param<bool>("test_enable_gps_imu", true)) {
        g2o::OptimizableGraph::Edge* edge;
        
        Eigen::Matrix2d information_matrix = Eigen::Matrix2d::Identity() / gps_edge_stddev_xy;
        edge = graph_slam->add_se2_prior_xy_edge(keyframe->node, xyz.head<2>(), information_matrix);
        
        graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("gps_edge_robust_kernel", "NONE"), private_nh.param<double>("gps_edge_robust_kernel_size", 1.0));
        
        updated = true; 
      }
    }

    auto remove_loc = std::upper_bound(gps_queue.begin(), gps_queue.end(), keyframes.back()->stamp, [=](const ros::Time& stamp, const geographic_msgs::GeoPointStampedConstPtr& geopoint) { return stamp < geopoint->header.stamp; });
    gps_queue.erase(gps_queue.begin(), remove_loc);
    return updated;
  }

  void imu_callback(const sensor_msgs::ImuPtr& imu_msg) {
    if(!enable_imu_orientation && !enable_imu_acceleration) {
      return;
    }

    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    imu_msg->header.stamp += ros::Duration(imu_time_offset);
    imu_queue.push_back(imu_msg);
  }

  bool flush_imu_queue() {
    std::lock_guard<std::mutex> lock(imu_queue_mutex);
    if(keyframes.empty() || imu_queue.empty() || base_frame_id.empty()) {
      return false;
    }

    bool updated = false;
    auto imu_cursor = imu_queue.begin();

    for(auto& keyframe : keyframes) {
      if(keyframe->stamp > imu_queue.back()->header.stamp) {
        break;
      }

      if(keyframe->stamp < (*imu_cursor)->header.stamp || keyframe->acceleration) {
        continue;
      }

      // find imu data which is closest to the keyframe
      auto closest_imu = imu_cursor;
      for(auto imu = imu_cursor; imu != imu_queue.end(); imu++) {
        auto dt = ((*closest_imu)->header.stamp - keyframe->stamp).toSec();
        auto dt2 = ((*imu)->header.stamp - keyframe->stamp).toSec();
        if(std::abs(dt) < std::abs(dt2)) {
          break;
        }

        closest_imu = imu;
      }

      imu_cursor = closest_imu;
      if(0.2 < std::abs(((*closest_imu)->header.stamp - keyframe->stamp).toSec())) {
        continue;
      }

      const auto& imu_ori = (*closest_imu)->orientation;
      const auto& imu_acc = (*closest_imu)->linear_acceleration;

      geometry_msgs::Vector3Stamped acc_imu;
      geometry_msgs::Vector3Stamped acc_base;
      geometry_msgs::QuaternionStamped quat_imu;
      geometry_msgs::QuaternionStamped quat_base;

      quat_imu.header.frame_id = acc_imu.header.frame_id = (*closest_imu)->header.frame_id;
      quat_imu.header.stamp = acc_imu.header.stamp = ros::Time(0);
      acc_imu.vector = (*closest_imu)->linear_acceleration;
      quat_imu.quaternion = (*closest_imu)->orientation;
      
      try {
        tf_listener.transformVector(base_frame_id, acc_imu, acc_base);
        tf_listener.transformQuaternion(base_frame_id, quat_imu, quat_base);
      } catch(std::exception& e) {
        std::cerr << "failed to find transform!!" << std::endl;
        return false;
      }
      
      keyframe->acceleration = Eigen::Vector2d(acc_base.vector.x, acc_base.vector.y);
      keyframe->orientation = Eigen::Rotation2D<double>(quatToAngle(Eigen::Quaterniond(quat_base.quaternion.w, quat_base.quaternion.x, quat_base.quaternion.y, quat_base.quaternion.z)));
      
       
      if(private_nh.param<bool>("test_enable_gps_imu", true)) {
        if(enable_imu_orientation) {
          Eigen::MatrixXd info = Eigen::MatrixXd::Identity(3, 3) / imu_orientation_edge_stddev;
          auto edge = graph_slam->add_se2_prior_quat_edge(keyframe->node, *keyframe->orientation, info);
          graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("imu_orientation_edge_robust_kernel", "NONE"), private_nh.param<double>("imu_orientation_edge_robust_kernel_size", 1.0));
        }
        
        updated = true;
      }
    }

    auto remove_loc = std::upper_bound(imu_queue.begin(), imu_queue.end(), keyframes.back()->stamp, [=](const ros::Time& stamp, const sensor_msgs::ImuConstPtr& imu) { return stamp < imu->header.stamp; });
    imu_queue.erase(imu_queue.begin(), remove_loc);
    return updated;
  }

  /**
   * @brief download buildings and add them to the graph
   * @param 
   * @return true if at least one node or edge has been added to the graph, false otherwise
   */
  bool update_buildings_nodes() {
    bool b_updated = false;
    bool new_kf = false; // true if at least there is a keyframe that has never been aligned

    sensor_msgs::NavSatFix tmp;
    tmp.header.frame_id = "map";
    tmp.header.stamp = ros::Time::now();
    tmp.latitude = zero_utm_lla(0);
    tmp.longitude = zero_utm_lla(1);
    tmp.altitude = zero_utm_lla(2);

    zero_utm_pub.publish(tmp);

    if(enter) { // test parameter
      for(auto& keyframe : keyframes) {
        // if the keyframe is never been aligned with map
        if(!keyframe->buildings_check && keyframe->index != 0) {
          new_kf = true;
          std::cout << "update_buildings_nodes" << std::endl;
          if(first_guess && !keyframe->utm_coord) {
            // if a keyframe has never been aligned since the start of the system, but there is no
            // gps coordinate associated to it, skip the keyframe (since for at least the first keyframe
            // we need a gps coord)
            continue; 
          }
          keyframe->buildings_check = true; // true if the keyframe has been checked for buildings
        
          /***************************************************************************************/
          // pre-processing on odom cloud
          // odomCloud contains lidar data
          pcl::PointCloud<PointT3>::Ptr odomCloud(new pcl::PointCloud<PointT3>); // cloud containing lidar data
          // temporary clouds
          pcl::PointCloud<PointT3>::Ptr temp_cloud(new pcl::PointCloud<PointT3>);
          pcl::PointCloud<PointT3>::Ptr temp_cloud_2(new pcl::PointCloud<PointT3>);
          pcl::PointCloud<PointT3>::Ptr temp_cloud_3(new pcl::PointCloud<PointT3>);
          pcl::PointCloud<PointT3>::Ptr temp_cloud_4(new pcl::PointCloud<PointT3>);
          pcl::PointCloud<PointT3>::Ptr temp_cloud_5(new pcl::PointCloud<PointT3>);
          pcl::copyPointCloud(*keyframe->cloud,*temp_cloud); // convert from pointxyzi to pointxyz

          // height filtering
          pcl::PassThrough<PointT3> pass;
          pass.setInputCloud (temp_cloud);
          pass.setFilterFieldName ("z");
          pass.setFilterLimits (ground_floor_max_thresh, 100.0);
          pass.filter(*temp_cloud_2);
          temp_cloud_2->header = (*keyframe->cloud).header;
          
          // downsampling
          pcl::Filter<PointT3>::Ptr downsample_filter;
          double downsample_resolution = private_nh.param<double>("downsample_resolution", 0.1);
          boost::shared_ptr<pcl::VoxelGrid<PointT3>> voxelgrid(new pcl::VoxelGrid<PointT3>());
          voxelgrid->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
          downsample_filter = voxelgrid;
          downsample_filter->setInputCloud(temp_cloud_2);
          downsample_filter->filter(*temp_cloud_3);
          temp_cloud_3->header = temp_cloud_2->header;
          
          // outlier removal
          pcl::RadiusOutlierRemoval<PointT3>::Ptr rad(new pcl::RadiusOutlierRemoval<PointT3>());
          rad->setRadiusSearch(radius_search);
          rad->setMinNeighborsInRadius(min_neighbors_in_radius);
          rad->setInputCloud(temp_cloud_3);
          rad->filter(*temp_cloud_4);
          temp_cloud_4->header = temp_cloud_3->header;
  
          // project the cloud on plane z=0
          pcl::ProjectInliers<PointT3> proj;
          pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
          coefficients->values.resize(4);
          coefficients->values[0]=0;
          coefficients->values[1]=0;
          coefficients->values[2]=1;
          coefficients->values[3]=0; 
          proj.setModelType(pcl::SACMODEL_PLANE); 
          proj.setInputCloud (temp_cloud_4);
          proj.setModelCoefficients (coefficients);
          proj.filter (*temp_cloud_5);
          temp_cloud_5->header = temp_cloud_4->header;
          
          pcl::transformPointCloud(*temp_cloud_5, *odomCloud, isometry2dto3d(keyframe->odom).matrix());
          odomCloud->header = temp_cloud_5->header;
          odomCloud->header.frame_id = "base_link";
          
           // publish original odom cloud
          sensor_msgs::PointCloud2Ptr oc_cloud_msg(new sensor_msgs::PointCloud2());
          pcl::toROSMsg(*keyframe->cloud, *oc_cloud_msg);
          oc_cloud_msg->header.frame_id = "base_link";
          oc_cloud_msg->header.stamp = keyframe->stamp;
          original_odom_pub.publish(oc_cloud_msg);
          // publish odom cloud
          sensor_msgs::PointCloud2Ptr o_cloud_msg(new sensor_msgs::PointCloud2());
          pcl::toROSMsg(*odomCloud, *o_cloud_msg);
          o_cloud_msg->header.frame_id = "odom";
          o_cloud_msg->header.stamp = keyframe->stamp;
          odom_pub.publish(o_cloud_msg);
          
          // additional pointcloud stored in the keyframe to be published later as map
          *keyframe->odomCloud = *temp_cloud_5;

          /***************************************************************************************/
          // compute dynamic lidar range (used to download buildings) if dynamic_lidar_range param 
          // is true, otherwise use the fixed one set in the initialization
          if(private_nh.param<bool>("dynamic_lidar_range", false)) {
            // computed as the mean of the distance between all points in the cloud and the centroid 
            // of the cloud
            std::cout << "set dynamic lidar range" << std::endl; 
            Eigen::Vector4f c;
            pcl::compute3DCentroid(*odomCloud, c);
            pcl::PointXYZ cpcl;
            cpcl.x = c(0);
            cpcl.y = c(1);
            cpcl.z = c(2);
            double sum = 0.0;
            for(int i = 0; i < odomCloud->size(); i++) {
              pcl::PointXYZ ptemp = odomCloud->at(i);
              double dist = pcl::euclideanDistance(cpcl, ptemp);
              sum += dist;
            }

            double dist = (sum/(odomCloud->size()));
            
            lidar_range = dist + private_nh.param<double>("lidar_range_add_factor", 3.0);
          }

          std::cout << "lidar range: " << lidar_range << std::endl;  
          /***************************************************************************************/  
          // compute the geographical coordinate associated with the keyframe, used to download buildings
          Eigen::Vector2d e_utm_coord = Eigen::Vector2d::Zero();
          int zone = 0;
          char band;
          if(!first_guess) {
            // for keyframes after the first, use the estimate as coordinate
            std::cout << "using est" << std::endl; 
            e_utm_coord = keyframe->node->estimate().translation(); 
            zone = zero_utm_zone;
            band = zero_utm_band;
          } else {
            // for the first keyframe ever examined use the gps coordinate instead of the estimate
            std::cout << "first guess" << std::endl; 
            e_utm_coord = (*keyframe->utm_coord);
            zone = *keyframe->utm_zone;
            band = *keyframe->utm_band;
          }
          
          Eigen::Vector2d e_zero_utm = (*zero_utm);
          geodesy::UTMPoint utm;
          // e_utm_coord are the coords of current keyframe wrt zero_utm, so to get real coords we add zero_utm
          utm.easting = e_utm_coord(0) + e_zero_utm(0);
          utm.northing = e_utm_coord(1) + e_zero_utm(1);
          utm.altitude = 0;
          utm.zone = zone;
          utm.band = band;
          geographic_msgs::GeoPoint lla = geodesy::toMsg(utm); // convert from utm to lla

          // download and parse buildings
          std::vector<Building> new_buildings = BuildingTools::getBuildings(lla.latitude, lla.longitude, lidar_range, e_zero_utm, private_nh.param<std::string>("buildings_host", "https://overpass-api.de"));
          if(new_buildings.size() > 0) { 
            // inside here if there are buildings (new or already seen)
            std::cout << "We found buildings! " << keyframe->index << std::endl;
            b_updated = true;
   
            std::vector<BuildingNode::Ptr> bnodes; // vector containing all buildings nodes for current kf (new and not new)
            // buildingsCloud is the cloud containing all buildings
            pcl::PointCloud<PointT3>::Ptr buildingsCloud(new pcl::PointCloud<PointT3>);
           
            // construct building nodes
            for(auto it2 = new_buildings.begin(); it2 != new_buildings.end(); it2++)
            {
              Building btemp = *it2;
              BuildingNode::Ptr bntemp(new BuildingNode());
              bntemp = get_building_node(btemp);
              if(bntemp == nullptr) { // enter if the building is new
                BuildingNode::Ptr bt(new BuildingNode());
                bt->building = btemp;
                bt->setReferenceSystem();
                // retrieve informations to build the se3 node
                // translation
                Eigen::Vector2d T = bt->local_origin; // local origin is already referring to zero_utm
                // rotation
                Eigen::Matrix2d R = Eigen::Matrix2d::Identity(); // gps coords don't give orientation
                // rototranslation
                Eigen::Isometry2d A;
                A.linear() = R;
                A.translation() = T;
                // set the node
                bt->node = graph_slam->add_se2_node(A); 
                *buildingsCloud += *(btemp.geometry);
                // buildings is a global array containing all the buildings ever seen from the start of the system
                buildings.push_back(bt);
                bnodes.push_back(bt);
              } else {
                /**********************************************************************************/
                // if true use the map of estimated buildings, otherwise always use the openstreemap ones
                if(private_nh.param<bool>("estimated_building_cloud", true)) {
                  // the building is transformed using its estimate
                  Eigen::Isometry3d est = isometry2dto3d(bntemp->node->estimate().toIsometry());
                  pcl::PointCloud<PointT3>::Ptr temp_cloud_7(new pcl::PointCloud<PointT3>);
                  pcl::transformPointCloud(*(bntemp->referenceSystem), *temp_cloud_7, est.matrix());
                  // buildingsCloud contains all the buildings
                  *buildingsCloud += *temp_cloud_7;
                } else {
                  // btemp.geometry always contains the openstreetmap geometry
                  *buildingsCloud += *(btemp.geometry);
                }
                
                /**********************************************************************************/
                bnodes.push_back(bntemp);
              }
            }
           
            buildingsCloud->header.frame_id = "map";
            // publish buildings cloud
            sensor_msgs::PointCloud2Ptr b_cloud_msg(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*buildingsCloud, *b_cloud_msg);
            b_cloud_msg->header.frame_id = "map";
            b_cloud_msg->header.stamp = keyframe->stamp;
            buildings_pub.publish(b_cloud_msg);

            // initial guess for the alignment
            Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
            if(first_guess) {
              // the first keyframe ever seen by the system uses the gps as initial guess
              std::cout << "first guess" << std::endl;
              guess.block<2,1>(0,3) = e_utm_coord.cast<float>();
              guess.block<2,2>(0,0) = (*(keyframe->orientation)).cast<float>().toRotationMatrix();
              guess = guess * ((isometry2dto3d(keyframe->odom)).cast<float>().matrix()).inverse();
              reset_counter = 0;
              //first_guess = false;
              //reset_gicp = false;
            } else {
              if(reset_gicp) {
                // use the keyframe estimate as initial guess
                std::cout << "reset gicp" << std::endl;
                guess = isometry2dto3d(keyframe->node->estimate().toIsometry()).cast<float>().matrix();
                guess = guess * ((isometry2dto3d(keyframe->odom)).cast<float>().matrix()).inverse();
                reset_counter = 0;
                //reset_gicp = false;
              } else {
                // use the previous estimate as initial guess
                std::cout << "prev guess" << std::endl;
                guess = prev_guess;
              }
            }
            std::cout << "guess: " << guess << std::endl;

            // all of this is used to correctly align bags with sparse buildings
            if((keyframe->accum_distance - dist_last_kf) > 200 && reset_gicp && !first_guess)
              reset_counter = 1;
            
            dist_last_kf = keyframe->accum_distance;

            // warp_fcn and te are used to constraints dof to get 2D working of the algorithm
            pcl::registration::WarpPointRigid3D<PointT3, PointT3>::Ptr warp_fcn(new pcl::registration::WarpPointRigid3D<PointT3,PointT3>);
            pcl::registration::TransformationEstimationLM<PointT3, PointT3>::Ptr te(new pcl::registration::TransformationEstimationLM<PointT3, PointT3>);
            te->setWarpFunction(warp_fcn);

            // first global alignment is done with ndt
            double ndt_resolution = private_nh.param<double>("ndt_resolution", 1.0);
            int num_threads = private_nh.param<int>("ndt_num_threads", 0);
            std::string nn_search_method = private_nh.param<std::string>("ndt_nn_search_method", "DIRECT7");
            std::cout << "registration: NDT_OMP" << std::endl;
            boost::shared_ptr<pclomp::NormalDistributionsTransform<PointT3, PointT3>> ndt(new pclomp::NormalDistributionsTransform<PointT3, PointT3>());
            if(num_threads > 0) {
              if(private_nh.param<bool>("enable_ndt_num_threads", false))
                ndt->setNumThreads(private_nh.param<int>("ndt_num_threads", 0));
            }
            if(private_nh.param<bool>("enable_ndt_transformation_epsilon", false))
              ndt->setTransformationEpsilon(private_nh.param<double>("ndt_transformation_epsilon", 0.1));
            if(private_nh.param<bool>("enable_ndt_maximum_iterations", false))
              ndt->setMaximumIterations(private_nh.param<int>("ndt_maximum_iterations", 35));
            if(private_nh.param<bool>("enable_ndt_step_size", false))
              ndt->setStepSize(private_nh.param<double>("ndt_step_size", 0.1));
            if(private_nh.param<bool>("enable_ndt_outlier_ratio", false))
              ndt->setOulierRatio(private_nh.param<double>("ndt_outlier_ratio", 0.55));

            if(private_nh.param<bool>("enable_ndt_resolution", false))
              ndt->setResolution(private_nh.param<double>("ndt_resolution", 1.0));
            if(private_nh.param<bool>("enable_ndt_search_method", false)) {
              if(nn_search_method == "KDTREE") {
                ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
              } else if(nn_search_method == "DIRECT1") {
                ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
              } else {
                ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
              }
            }

            /*
            // used eventually to dynamically set the gicp max distance in case of bags with sparse buildings
            if(reset_counter == 1) {
              gicp->setMaxCorrespondenceDistance(private_nh.param<double>("gicp_max_correspondance_distance", 15.0));
              std::cout << "setting to " << private_nh.param<double>("gicp_max_correspondance_distance", 15.0) << std::endl;
            } else if(reset_counter == 2) {
              gicp->setMaxCorrespondenceDistance(private_nh.param<double>("gicp_max_correspondance_distance_mid", 2.0));
              std::cout << "setting to " << private_nh.param<double>("gicp_max_correspondance_distance_mid", 2.0) << std::endl;
            } else if(reset_counter == 3) {
              gicp->setMaxCorrespondenceDistance(private_nh.param<double>("gicp_max_correspondance_distance_mid_2", 3.0));
              std::cout << "setting to " << private_nh.param<double>("gicp_max_correspondance_distance_mid_2", 3.0) << std::endl;
            } else {
              gicp->setMaxCorrespondenceDistance(private_nh.param<double>("gicp_max_correspondance_distance_min", 1.0));
              std::cout << "setting to " << private_nh.param<double>("gicp_max_correspondance_distance_min", 1.0) << std::endl;
            }*/
            
            ndt->setTransformationEstimation(te);
            
            ndt->setInputTarget(buildingsCloud);
            ndt->setInputSource(odomCloud);
            pcl::PointCloud<PointT3>::Ptr aligned(new pcl::PointCloud<PointT3>());
            // align scan to buildings
            ndt->align(*aligned, guess);
            std::cout << "has converged:" << ndt->hasConverged() << " score: " << ndt->getFitnessScore() << std::endl;
            Eigen::Matrix4f transformation = ndt->getFinalTransformation();
            std::cout<< "Transformation: " << transformation << std::endl;
            prev_guess = transformation;

            // publish ndt resulting transform
            aligned->header.frame_id = "map";
            transformed_pub.publish(aligned);

            // convert to 2D the transformation from ndt (t_s_bs = transformation scan to buildings)
            Eigen::Matrix3d t_s_bs = matrix4dto3d(transformation.cast<double>());
         
            // compute information matrix of buildings
            Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(3, 3);
            if(private_nh.param<bool>("b_use_const_inf_matrix", false)) {
              // constant im
              information_matrix.topLeftCorner(2, 2).array() /= private_nh.param<float>("building_edge_stddev_xy", 0.25);
              information_matrix(2, 2) /= private_nh.param<float>("building_edge_stddev_q", 1);
            } else {
              // dynamic im, computed as 3D and then converted to 2D
              Eigen::MatrixXd information_matrix_6 = Eigen::MatrixXd::Identity(6, 6);

              Eigen::Isometry3d t_s_bs_iso = Eigen::Isometry3d::Identity();
              t_s_bs_iso.matrix() = transformation.cast<double>(); 
              information_matrix_6 = inf_calclator->calc_information_matrix_buildings(buildingsCloud, odomCloud, t_s_bs_iso);
            
              information_matrix.block<2,2>(0,0) = information_matrix_6.block<2,2>(0,0);
              information_matrix(2,2) = information_matrix_6(5,5);
            }
            std::cout << "BUILDINGS INF: " << information_matrix << std::endl;

            // convert ndt transformation to isometry (needed later)
            Eigen::Isometry2d t_s_bs_iso = Eigen::Isometry2d::Identity();
            t_s_bs_iso.matrix() = t_s_bs;

            /*********************************************************************************************/
            /*
            // all of this eventually needed to set dynamic parameters used with sparse buildings bags.
            // currently disabled.
            // reset_counter is tuned wrt the fitness score. 
            double ft = InformationMatrixCalculator::calc_fitness_score_buildings(buildingsCloud, odomCloud, isometry2dto3d(t_s_bs_iso), 1.0);
            if(ft > 0.3) {
              if(reset_counter == 1) {
                reset_counter = 1;
                //std::cout << "fitness score too high 0.3 = 1" << std::endl;
              }
              else if(ft > 0.35) {
                reset_counter = 3;
              } else { 
                reset_counter = 2;
                //std::cout << "fitness score too high 0.3 = 2" << std::endl;
              }
            // it was here
            } else {
              reset_counter = 0;
              //std::cout << "fitness score low = 0" << std::endl;
            }*/

            /*} else if(ft > 0.25) {
              if(reset_counter == 1) {
                reset_counter = 1;
                std::cout << "fitness score too high 0.25 = 1" << std::endl;
              } else {
                reset_counter = 2;
                std::cout << "fitness score too high 0.25 = 2" << std::endl;
              }*/

            /********************************************************************************/
            Eigen::Isometry2d temp = t_s_bs_iso*(keyframe->odom);

            // setting up gicp for the local alignment that will be done later
            std::cout << "registration: GICP_OMP" << std::endl;
            boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<PointT3, PointT3>> gicp(new pclomp::GeneralizedIterativeClosestPoint<PointT3, PointT3>());

            if(private_nh.param<bool>("enable_gicp_transformation_epsilon", true))
              gicp->setTransformationEpsilon(private_nh.param<double>("gicp_transformation_epsilon", 0.01));
            if(private_nh.param<bool>("enable_gicp_maximum_iterations", true))
              gicp->setMaximumIterations(private_nh.param<int>("gicp_maximum_iterations", 64));
            if(private_nh.param<bool>("enable_gicp_use_reciprocal_correspondences", true))
              gicp->setUseReciprocalCorrespondences(private_nh.param<bool>("gicp_use_reciprocal_correspondences", false));
            if(private_nh.param<bool>("enable_gicp_correspondence_randomness", true))
              gicp->setCorrespondenceRandomness(private_nh.param<int>("gicp_correspondence_randomness", 20));
            if(private_nh.param<bool>("enable_gicp_max_optimizer_iterations", true))
              gicp->setMaximumOptimizerIterations(private_nh.param<int>("gicp_max_optimizer_iterations", 20));

            if(private_nh.param<bool>("enable_gicp_max_correspondance_distance", false))
              gicp->setMaxCorrespondenceDistance(private_nh.param<double>("gicp_max_correspondance_distance", 0.05));
            if(private_nh.param<bool>("enable_gicp_euclidean_fitness_epsilon", false))
              gicp->setEuclideanFitnessEpsilon(private_nh.param<double>("gicp_euclidean_fitness_epsilon", 1));
            if(private_nh.param<bool>("enable_gicp_ransac_outlier_threshold", false)) 
              gicp->setRANSACOutlierRejectionThreshold(private_nh.param<double>("gicp_ransac_outlier_threshold", 1.5));

            if(private_nh.param<bool>("enable_gicp_rotation_epsilon", false)) 
              gicp->setRotationEpsilon(private_nh.param<double>("gicp_rotation_epsilon", 0.002));

            // print the keyframe estimate in map frame
            if(private_nh.param<bool>("print_tf", false)) {
              geometry_msgs::TransformStamped ts = matrix2transform2d(keyframe->stamp, (keyframe->node->estimate().toIsometry().matrix()).cast<float>(), "map", "kf_"+std::to_string(keyframe->id()));
              b_tf_broadcaster.sendTransform(ts);
            }

            // iterate over buildings seen by this keyframe
            for(auto it2 = bnodes.begin(); it2 != bnodes.end(); it2++)
            {
              BuildingNode::Ptr bntemp = *it2;

              // retrieve the pointcloud associated to the current building (estimated or not)
              pcl::PointCloud<PointT3>::Ptr temp_cloud_building(new pcl::PointCloud<PointT3>);
              if(private_nh.param<bool>("estimated_building_cloud", true)) {
                Eigen::Isometry3d est = isometry2dto3d(bntemp->node->estimate().toIsometry());
                pcl::transformPointCloud(*(bntemp->referenceSystem), *temp_cloud_building, est.matrix());
              } else {
                *temp_cloud_building = *(bntemp->building.geometry);
              }      

              
              // set up gicp and perform local alignment between the entire lidar scan and the building pointcloud
              gicp->setInputTarget(temp_cloud_building);
              gicp->setInputSource(odomCloud);
              pcl::PointCloud<PointT3>::Ptr b_aligned(new pcl::PointCloud<PointT3>());

              // initial guess for local alignment is the tf from global alignment
              Eigen::Matrix4f guess_tmp = Eigen::Matrix4f::Identity();
              guess_tmp = transformation;
              gicp->align(*b_aligned, guess_tmp);
              
              std::cout << "b " << bntemp->node->id() << " has converged: " << gicp->hasConverged() << " score: " << gicp->getFitnessScore() << std::endl;
              Eigen::Matrix4f t_scan_buildings_4f = gicp->getFinalTransformation();
              // publish odomCloud aligned to the building
              b_aligned->header.frame_id = "map";
              temp_pub.publish(b_aligned);

              // various conversions of the local transformation
              Eigen::Matrix3d t_scan_buildings = matrix4dto3d(t_scan_buildings_4f.cast<double>());
              Eigen::Isometry2d t_scan_buildings_iso = Eigen::Isometry2d::Identity();
              t_scan_buildings_iso.matrix() = t_scan_buildings;

              /****************************/
              // retrieve correspondences used to compute the information matrix
              boost::shared_ptr<std::vector<int>> src_corr(new std::vector<int>()), tgt_corr(new std::vector<int>());
              inf_calclator->calc_fitness_score_buildings(src_corr, tgt_corr, temp_cloud_building, odomCloud, isometry2dto3d(t_scan_buildings_iso),1.0);
              /****************************/

              // t_map_building = building node estimate (hyphotethical "building" frame) that is in map frame
              Eigen::Matrix3d t_map_building = Eigen::Matrix3d::Identity();
              t_map_building = bntemp->node->estimate().toIsometry().matrix();

              // print t_map_building
              if(private_nh.param<bool>("print_b_tf", false)) {
                geometry_msgs::TransformStamped ts3 = matrix2transform2d(keyframe->stamp,  (t_map_building).cast<float>(), "map", "b_"+std::to_string(bntemp->node->id()));
                b_tf_broadcaster.sendTransform(ts3);
              }

              // t_prior_kf = corrected keyframe tf using buildings (intermediate tf)
              Eigen::Matrix3d t_prior_kf = t_scan_buildings*keyframe->odom.matrix();
              
              // t_scan_building = tf between the keyframe node and building node
              Eigen::Matrix3d t_scan_building = (t_map_building.inverse())*t_prior_kf;
              Eigen::Isometry2d t_scan_building_iso = Eigen::Isometry2d::Identity();
              t_scan_building_iso.matrix() = t_scan_building;

              // compute information matrix
              Eigen::Matrix3d im = Eigen::Matrix3d::Identity();
              if(private_nh.param<bool>("b_use_const_inf_matrix", true)) {
                //constant im
                im.topLeftCorner(2, 2).array() /= private_nh.param<double>("building_edge_stddev_xy", 1.0);
                im(2, 2) /= private_nh.param<double>("building_edge_stddev_q", 0.1);
              } 
              else {
                //dynamic im
                /*****************************************************************************/
                // compute the information matrix for the specific building

                // get the estimated building pointcloud
                Eigen::Isometry3d est = isometry2dto3d(bntemp->node->estimate().toIsometry());
                pcl::PointCloud<PointT3>::Ptr b_pcl(new pcl::PointCloud<PointT3>);
                pcl::transformPointCloud(*(bntemp->referenceSystem), *b_pcl, est.matrix());
                
                int k = 0;
                double mean_dist = 0.0;
            
                

                
                for(int i = 0; i < tgt_corr->size(); i++) {
                  
                  PointT3 pt = temp_cloud_building->at(tgt_corr->at(i));
                  

                    // compute the squared distance (squared because in pcl the fitness score is computed on the squared distance)
                    double dx = (aligned->at(src_corr->at(i))).x - pt.x;
                   
                    double dy = (aligned->at(src_corr->at(i))).y - pt.y;
                    
                    double dist = dx*dx + dy*dy;
                   

                   mean_dist += dist;
                   k++;
                   
               
                }

                double temp_ft = 0.0;
               
                if(k > 0) {
                  // compute fitness score
                  temp_ft = (mean_dist/k);
                } 
                std::cout << "ft: " << temp_ft << std::endl;

                

                Eigen::MatrixXd im = Eigen::MatrixXd::Identity(3, 3);
                if(k > 0) {
                  // compute information matrix based on fitness score
                  double x = temp_ft;

                  // read params for computation of information matrix
                  double a = private_nh.param<double>("b_var_gain_a", 20.0);
                  
                  double min_y = std::pow(private_nh.param<double>("b_min_stddev_x", 0.1), 2);
                  
                  double max_y = std::pow(private_nh.param<double>("b_max_stddev_x", 5.0), 2);
                  
                  double min_y_q = std::pow(private_nh.param<double>("b_min_stddev_q", 0.05), 2);
                 
                  double max_y_q = std::pow(private_nh.param<double>("b_max_stddev_q", 0.2), 2);
                 

                  double max_x = private_nh.param<double>("b_fitness_score_thresh", 0.5);
                  
                  double y = x*max_x; // linear proportion

                  double w_x = min_y + (max_y - min_y) * y;
                  double w_q = min_y_q + (max_y_q - min_y_q) * y;

                  // information matrix calculation
                  im.topLeftCorner(2, 2).array() /= w_x;
                  im(2, 2) /= w_q;
                }
               
                /*****************************************************************************/
                
              }
              
              if((src_corr->size() > private_nh.param<double>("k_thresh", 0)) && gicp->hasConverged()) {
                // add edge only if there are enough correspondences and if local alignment has converged
                auto edge = graph_slam->add_se2_edge(bntemp->node, keyframe->node, t_scan_building_iso, im);
                graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("map_edge_robust_kernel", "NONE"), private_nh.param<double>("map_edge_robust_kernel_size", 1.0)); 
              }
              
            }
            
            // used for dynamic setting of max dist in case of sparse buildings bags
            first_guess = false;
            reset_gicp = false;

            keyframe->buildings_nodes = bnodes; // store the nodes associated to a keyframe in it
          } else {

            std::cout << "No buildings found!" << std::endl;
            b_updated = false;
          }
        } 
      }
    }

    // if there are new keyframes, but all of them do not have buildings (b_updated = false) 
    // then re-initialize the first guess for next keyframe that will have buildings.
    // used for bags that have scarse buildings to dynamically tune parameters.
    if(new_kf && !b_updated) {
      std::cout << "RE-INITIATE INITIAL GUESS!!!!!!!!!!!!!" << std::endl;
      fix_first_building = true;
      reset_gicp = true;
    }
    return b_updated;
  }

  /**
   * @brief Given a Building object, retrieve the corresponding BuildingNode from the "buildings" global array
   * @param Building object
   * @return BuildingNode object corresponding to the input Building object
   */
  BuildingNode::Ptr get_building_node(Building b) {
    for(auto it = buildings.begin(); it != buildings.end(); it++)
    {
      BuildingNode::Ptr bntemp = *it;
      Building btemp = bntemp->building;
      if(btemp.id.compare(b.id) == 0)
        return bntemp;
    }
    return nullptr;
  }

  /**
   * @brief Load ground truth poses (as provided by Kitti) 
   * @param name of the file containing the gt poses
   * @return a vector of 3D ground truth poses
   */
  std::vector<Eigen::Matrix4d> loadPoses(std::string file_name) {
    std::vector<Eigen::Matrix4d> poses;
    FILE *fp = fopen(file_name.c_str(),"r");
    if (!fp)
      return poses;
    while (!feof(fp)) {
      Eigen::Matrix4d P = Eigen::Matrix4d::Identity();
      if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                     &P(0,0), &P(0,1), &P(0,2), &P(0,3),
                     &P(1,0), &P(1,1), &P(1,2), &P(1,3),
                     &P(2,0), &P(2,1), &P(2,2), &P(2,3) )==12) {
        Eigen::Matrix3d pose_3d = Eigen::Matrix3d::Identity();
        pose_3d = matrix4dto3d(P);
        poses.push_back(P);
      }
    }
    fclose(fp);
    return poses;
  }

  /**
   * @brief generate map point cloud and publish it
   * @param event
   */
  void map_points_publish_timer_callback(const ros::WallTimerEvent& event) {
    if(!map_points_pub.getNumSubscribers() || !graph_updated) {
      return;
    }

    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, map_cloud_resolution);
    if(!cloud) {
      return;
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud, *cloud_msg);

    map_points_pub.publish(cloud_msg);
  }

  /**
   * @brief this methods adds all the data in the queues to the pose graph, and then optimizes the pose graph
   * @param event
   */
  void optimization_timer_callback(const ros::WallTimerEvent& event) {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    // add keyframes and floor coeffs in the queues to the pose graph
    bool keyframe_updated = flush_keyframe_queue();

    if(!keyframe_updated) {
      std_msgs::Header read_until;
      read_until.stamp = ros::Time::now() + ros::Duration(30, 0);
      read_until.frame_id = points_topic;
      read_until_pub.publish(read_until);
      read_until.frame_id = "/filtered_points";
      read_until_pub.publish(read_until);
    }
    
    // added the call to update_buildings_nodes to download and insert buildings nodes and edges
    if(!keyframe_updated & !flush_gps_queue() & !flush_imu_queue() & !update_buildings_nodes()) {
      return;
    }
    
    // loop detection
    std::vector<Loop::Ptr> loops = loop_detector->detect(keyframes, new_keyframes, *graph_slam);
    for(const auto& loop : loops) {
      Eigen::Isometry2d relpose(loop->relative_pose.cast<double>());
      // also here information matrix computation has been left in 3D and then the information matrix has
      // been converted to 2D
      Eigen::MatrixXd information_matrix_6 = inf_calclator->calc_information_matrix(loop->key1->cloud, loop->key2->cloud, isometry2dto3d(relpose));
      Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(3, 3);
      information_matrix.block<2,2>(0,0) = information_matrix_6.block<2,2>(0,0);
      information_matrix(2,2) = information_matrix_6(5,5);

      auto edge = graph_slam->add_se2_edge(loop->key1->node, loop->key2->node, relpose, information_matrix);
      graph_slam->add_robust_kernel(edge, private_nh.param<std::string>("loop_closure_edge_robust_kernel", "NONE"), private_nh.param<double>("loop_closure_edge_robust_kernel_size", 1.0));
    }

    std::copy(new_keyframes.begin(), new_keyframes.end(), std::back_inserter(keyframes));
    new_keyframes.clear();

    // move the first node anchor position to the current estimate of the first node pose
    // so the first node moves freely while trying to stay around the origin
    if(anchor_node && private_nh.param<bool>("fix_first_node_adaptive", true)) {
      Eigen::Isometry2d anchor_target = static_cast<g2o::VertexSE2*>(anchor_edge->vertices()[1])->estimate().toIsometry();
      anchor_node->setEstimate(anchor_target);
    }
    //std::cout << "before opt" << std::endl;
   
    // optimize the pose graph
    int num_iterations = private_nh.param<int>("g2o_solver_num_iterations", 1024);
    graph_slam->optimize(num_iterations);
    
    // publish tf
    const auto& keyframe = keyframes.back();
    Eigen::Isometry2d trans = keyframe->node->estimate().toIsometry() * keyframe->odom.inverse();
    trans_odom2map_mutex.lock();
    trans_odom2map = trans.matrix().cast<float>();
    trans_odom2map_mutex.unlock();

    std::vector<KeyFrameSnapshot::Ptr> snapshot(keyframes.size());
    std::transform(keyframes.begin(), keyframes.end(), snapshot.begin(), [=](const KeyFrame::Ptr& k) { return std::make_shared<KeyFrameSnapshot>(k); });

    keyframes_snapshot_mutex.lock();
    keyframes_snapshot.swap(snapshot);
    keyframes_snapshot_mutex.unlock();
    graph_updated = true;

    if(odom2map_pub.getNumSubscribers()) {
      geometry_msgs::TransformStamped ts = matrix2transform2d(keyframe->stamp, trans.matrix().cast<float>(), map_frame_id, odom_frame_id);
      odom2map_pub.publish(ts);
    }

    if(markers_pub.getNumSubscribers()) {
      auto markers = create_marker_array(ros::Time::now());
      markers_pub.publish(markers);
    }

    /*****************************************************************************/
    // publish clouds:
    // estimatedBuildingsCloud contains estimated buildings (pink)
    // buildingsCloud contains OpenStreetMap buildings (green)
    pcl::PointCloud<PointT3>::Ptr estimatedBuildingsCloud(new pcl::PointCloud<PointT3>);
    pcl::PointCloud<PointT3>::Ptr buildingsCloud(new pcl::PointCloud<PointT3>);
    for(auto it3 = buildings.begin(); it3 != buildings.end(); it3++)
    {
      BuildingNode::Ptr btemp = *it3;
      Eigen::Isometry3d est = isometry2dto3d(btemp->node->estimate().toIsometry());
      pcl::PointCloud<PointT3>::Ptr temp_cloud_7(new pcl::PointCloud<PointT3>);
      pcl::transformPointCloud(*(btemp->referenceSystem), *temp_cloud_7, est.matrix());
      *estimatedBuildingsCloud += *temp_cloud_7;

      *buildingsCloud += *btemp->building.geometry;
    }

    // publish estimatedBuildingsCloud
    sensor_msgs::PointCloud2Ptr eb_cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*estimatedBuildingsCloud, *eb_cloud_msg);
    eb_cloud_msg->header.frame_id = "map";
    eb_cloud_msg->header.stamp = keyframe->stamp;
    estimated_buildings_pub.publish(eb_cloud_msg);

    // publish buildingsCloud
    sensor_msgs::PointCloud2Ptr all_b_cloud_msg(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*buildingsCloud, *all_b_cloud_msg);
    all_b_cloud_msg->header.frame_id = "map";
    all_b_cloud_msg->header.stamp = keyframe->stamp;
    all_buildings_pub.publish(all_b_cloud_msg);
    
    /********************************************************************************************/
    // errors computation
    // RPE always computed
    // ATE computed only if the parameter "enable_ate_calculation" is true (because gt is not always present)
    std::cout << "starting errors computation" << std::endl;
    // settings to correctly print to file
    // all files can be found in the .ros temporary folder in the "home" folder on Ubuntu
    Eigen::IOFormat lineFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, " ", " ", "", "", "", "");

    // since Kitti gt are in camera_gray_left but estimates are in base_link, retrieve the tf between
    // base_link and camera_gray_left to convert gt to base_link or viceversa
    tf::StampedTransform transform_t;
    tf_listener.lookupTransform("base_link", "camera_gray_left", ros::Time(0), transform_t);
    Eigen::Quaterniond q;
    tf::quaternionTFToEigen(transform_t.getRotation(), q);
    Eigen::Vector3d v;
    tf::vectorTFToEigen(transform_t.getOrigin(), v);
    Eigen::Matrix4d base_camera = Eigen::Matrix4d::Identity();
    base_camera.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    
    // ATE start
      // retrieve the position in the "keyframes" array of the first arrived keyframe
      int index = -1, k = 0;
      while(k < keyframes.size()) {
        index = getIndexPosition(k);
        if(index >=0)
          break;
        k++;
      }
      if(index == -1)
        index = 0;

      // estimate of the first arrived keyframe
      Eigen::Matrix4d estimate = isometry2dto3d(keyframes[index]->node->estimate().toIsometry()).matrix();

      Eigen::Matrix4d tr = estimate*base_camera;

      // delta transforms gt to keyframes, used for visualization
      Eigen::Matrix4d delta = Eigen::Matrix4d::Identity();
      delta = tr * (gt[index].inverse());
      //std::cout << "delta: " << delta << std::endl;
      
      // delta2 transforms keyframes to gt, used for error calculation
      Eigen::Matrix4d delta_2 = Eigen::Matrix4d::Identity();
      delta_2 = gt[index] * (tr.inverse());
      
      if(private_nh.param<bool>("enable_ate_calculation", true)) {
      // on file "align.txt" write "delta" and "delta_2" matrices
      std::ofstream myfile5;
      myfile5.open("align.txt");
      myfile5 << delta.format(lineFmt) << std::endl;
      myfile5 << delta_2.format(lineFmt) << std::endl;
      myfile5.close();
      
      // publish gt markers
      visualization_msgs::Marker gt_traj_marker;
      gt_traj_marker.header.frame_id = "map";
      gt_traj_marker.header.stamp = ros::Time::now();
      gt_traj_marker.ns = "gt";
      gt_traj_marker.id = 0;
      gt_traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;
      gt_traj_marker.action = visualization_msgs::Marker::ADD;

      gt_traj_marker.pose.orientation.w = 1.0;
      gt_traj_marker.scale.x = gt_traj_marker.scale.y = gt_traj_marker.scale.z = 0.7;

      std::ofstream myfile;
      // "poses.txt" contains estimated poses
      myfile.open ("poses.txt");
      std::ofstream myfile2;
      // "gt.txt" contains gt poses corresponding to the estimated poses in "base_link" rf
      myfile2.open ("gt.txt");
      std::ofstream myfile3;
      // "dist.txt" contains the euclidean distance between the position of the estimated pose and corresponding gt
      myfile3.open ("dist.txt");
      std::ofstream myfile4;
      // "ate.txt" contains the ate as mean +- variance
      myfile4.open ("ate.txt");
      double sum = 0.0;
      double sum_sq = 0.0;
      int j = 0;

      for(int i = 0; i < gt.size(); i++) {
        // given a gt pose index in the "gt" array, get the index in the "keyframes" array
        // of the keyframe corresponding to that gt pose (given by the keyframe "ii" order of arrival)
        int pos = getIndexPosition(i);
        if(pos >= 0) {
          // there is a keyframe for that ground truth pose
          j++;
          
          // pose = keyframe pose, converted to 3D to be compared with gt
          Eigen::Matrix4d pose = isometry2dto3d(keyframes[pos]->node->estimate().toIsometry()).matrix();
          
          Eigen::Matrix4d pose_trans = pose;
          Eigen::Matrix4d pose_error = (state6to3(delta*gt[i]).inverse())*pose_trans;

          double dx = pose_error(0, 3);
          
          double dy = pose_error(1, 3);
          
          double dist = sqrt(dx*dx+dy*dy);
          sum += dist;
          sum_sq += (dist*dist);
          
          myfile << pose.format(lineFmt) << std::endl;
          myfile2 << state6to3(gt[i]).format(lineFmt) << std::endl;
          myfile3 << i << " " << dist << std::endl;
        }  
      }

      // ate
      double t_mean = sum/(j);
      double t_variance = sqrt((sum_sq/(j))-(t_mean*t_mean));

      // publish ground truth markers
      for(int i = 0; i < gt.size(); i++) {
        geometry_msgs::Point p;

        p.x = (delta * gt[i])(0,3);
        p.y = (delta * gt[i])(1,3);
        p.z = 0;
        gt_traj_marker.points.push_back(p);
        std_msgs::ColorRGBA c;
        c.r = 0.0;
        c.g = 1.0;
        c.b = 0.0;
        c.a = 1.0;
        gt_traj_marker.colors.push_back(c);
      }

      gt_markers_pub.publish(gt_traj_marker);

      std::cout << "ate t_mean: " << t_mean << std::endl;
      std::cout << "ate t_variance: " << t_variance << std::endl;

      myfile4 << t_mean << " " << t_variance << std::endl;

      myfile.close();
      myfile2.close();
      myfile3.close();
      myfile4.close();
    }

    // RPE start
    Eigen::Matrix4d prev_pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d prev_gt = Eigen::Matrix4d::Identity();
    // rpe_poses will contain the relative pose between consecutive estimated poses
    std::vector<Eigen::Matrix4d> rpe_poses;
    // rpe_gt will containt the relative pose between consecutive gt poses
    std::vector<Eigen::Matrix4d> rpe_gt;
    std::ofstream myfile6;
    myfile6.open ("rpe.txt");
    bool first_enter = true;
    // populate rpe_poses and rpe_gt
    for(int i = 0; i < gt.size(); i++) {
      int pos = getIndexPosition(i);
      if(pos >= 0) {
        Eigen::Matrix4d pose = isometry2dto3d(keyframes[pos]->node->estimate().toIsometry()).matrix();
        if(!first_enter) {
          rpe_poses.push_back(computeRelativeTrans(prev_pose, (pose)));
          rpe_gt.push_back(computeRelativeTrans(prev_gt, state6to3(delta*gt[i])));
        } 
        first_enter = false;
        prev_pose = pose;
        prev_gt = state6to3(delta*gt[i]);
      }
    }
    
    double t_rpe_sum = 0.0;
    double r_rpe_sum = 0.0;
    for(int i = 0; i < rpe_poses.size(); i++) {
      // compute difference between relarive rpe pose and relative estimated pose
      Eigen::Matrix4d rpe_delta = computeRelativeTrans(rpe_gt[i], rpe_poses[i]);
      
      // t_rpe
      double t = sqrt((rpe_delta(0,3)*rpe_delta(0,3))+(rpe_delta(1,3)*rpe_delta(1,3)));
      // r_rpe
      double angle_temp = (((rpe_delta(0,0) + rpe_delta(1,1) + 1)-1)/2);
      if(angle_temp > 1.0)
        angle_temp = 1.0;
      if(angle_temp < -1.0)
        angle_temp = -1.0;
      double angle = std::acos(angle_temp); 
    
      t_rpe_sum += t;
      r_rpe_sum += angle;
    }
    
    if(rpe_poses.size() > 0) {
      // final r_rpe and t_rpe
      double t_rpe = sqrt(t_rpe_sum/(rpe_poses.size()));
      double r_rpe = sqrt(r_rpe_sum/(rpe_poses.size()));
      myfile6 << t_rpe << " " << r_rpe << std::endl;
      std::cout << "t_rpe: " << t_rpe << std::endl;
      std::cout << "r_rpe: " << r_rpe << std::endl;
    }
    myfile6.close();
    
    // RPE end
    std::cout << "finished error computation" << std::endl;
    /****************************************************************************************/
  }

  /**
   * @brief compute relative transformation between two poses
   * @param pose1, pose2
   * @return relative transformation between pose1 and pose2
   */
  Eigen::Matrix4d computeRelativeTrans(Eigen::Matrix4d pose1, Eigen::Matrix4d pose2) {
    Eigen::Matrix4d delta = (pose1.inverse())*pose2;
    return delta;
  }

  /**
   * @brief get the position into the "keyframes" array of keyframe with index (ii) "index"
   * @param index (ii) to be searched
   * @return index in the "keyframes" array, -1 if no keyframe with that index (ii) is found
   */
  int getIndexPosition(int index) {
    for(int i = 0; i < keyframes.size(); i++) {
      if(keyframes[i]->index==index) {
        return i;
      }
    }
    return -1;
  }

  /**
   * @brief create visualization marker
   * @param stamp
   * @return
   */
  visualization_msgs::MarkerArray create_marker_array(const ros::Time& stamp) const {
    // modified to print 2D values and buildings (pink dots)
    visualization_msgs::MarkerArray markers;
    markers.markers.resize(4);

    // node markers
    visualization_msgs::Marker& traj_marker = markers.markers[0];
    traj_marker.header.frame_id = "map";
    traj_marker.header.stamp = stamp;
    traj_marker.ns = "nodes";
    traj_marker.id = 0;
    traj_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    traj_marker.pose.orientation.w = 1.0;
    traj_marker.scale.x = traj_marker.scale.y = traj_marker.scale.z = 0.7;

    visualization_msgs::Marker& imu_marker = markers.markers[1];
    imu_marker.header = traj_marker.header;
    imu_marker.ns = "imu";
    imu_marker.id = 1;
    imu_marker.type = visualization_msgs::Marker::SPHERE_LIST;

    imu_marker.pose.orientation.w = 1.0;
    imu_marker.scale.x = imu_marker.scale.y = imu_marker.scale.z = 0.75;

    traj_marker.points.resize(keyframes.size() + buildings.size());
    traj_marker.colors.resize(keyframes.size() + buildings.size());
   
    for(int i = 0; i < keyframes.size(); i++) {
      Eigen::Vector2d pos = keyframes[i]->node->estimate().translation();
      traj_marker.points[i].x = pos.x();
      traj_marker.points[i].y = pos.y();
      traj_marker.points[i].z = 0;

      double p = static_cast<double>(i) / keyframes.size();
      traj_marker.colors[i].r = 1.0;
      traj_marker.colors[i].g = 0.0;
      traj_marker.colors[i].b = 0.0;
      traj_marker.colors[i].a = 1.0;

      if(keyframes[i]->acceleration) {
        Eigen::Vector2d pos = keyframes[i]->node->estimate().translation();
        geometry_msgs::Point point;
        point.x = pos.x();
        point.y = pos.y();
        point.z = 0;

        std_msgs::ColorRGBA color;
        color.r = 0.0;
        color.g = 0.0;
        color.b = 1.0;
        color.a = 0.1;

        imu_marker.points.push_back(point);
        imu_marker.colors.push_back(color);
      }
    }

     for(int i = 0; i < buildings.size(); i++) {
       if(buildings[i]->node != nullptr) {
        
        Eigen::Isometry2d pos = buildings[i]->node->estimate().toIsometry();
        traj_marker.points[i+keyframes.size()].x = pos.translation().x();
        traj_marker.points[i+keyframes.size()].y = pos.translation().y();
        traj_marker.points[i+keyframes.size()].z = 0;

        traj_marker.colors[i+keyframes.size()].r = 255.0/255.0;
        traj_marker.colors[i+keyframes.size()].g = 0.0/255.0;
        traj_marker.colors[i+keyframes.size()].b = 255.0/255.0;
        traj_marker.colors[i+keyframes.size()].a = 1.0;
      }
     
    }
    
    // edge markers
    visualization_msgs::Marker& edge_marker = markers.markers[2];
    edge_marker.header.frame_id = "map";
    edge_marker.header.stamp = stamp;
    edge_marker.ns = "edges";
    edge_marker.id = 2;
    edge_marker.type = visualization_msgs::Marker::LINE_LIST;

    edge_marker.pose.orientation.w = 1.0;
    edge_marker.scale.x = 0.05;

    edge_marker.points.resize(graph_slam->graph->edges().size() * 2);
    edge_marker.colors.resize(graph_slam->graph->edges().size() * 2);

    auto edge_itr = graph_slam->graph->edges().begin();
    for(int i = 0; edge_itr != graph_slam->graph->edges().end(); edge_itr++, i++) {
      g2o::HyperGraph::Edge* edge = *edge_itr;
      g2o::EdgeSE2* edge_se3 = dynamic_cast<g2o::EdgeSE2*>(edge);
      if(edge_se3) {
        g2o::VertexSE2* v1 = dynamic_cast<g2o::VertexSE2*>(edge_se3->vertices()[0]);
        g2o::VertexSE2* v2 = dynamic_cast<g2o::VertexSE2*>(edge_se3->vertices()[1]);
        Eigen::Vector2d pt1 = v1->estimate().translation();
        Eigen::Vector2d pt2 = v2->estimate().translation();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = 0;
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = 0;

        double p1 = static_cast<double>(v1->id()) / graph_slam->graph->vertices().size();
        double p2 = static_cast<double>(v2->id()) / graph_slam->graph->vertices().size();
        edge_marker.colors[i * 2].r = 0.0;
        edge_marker.colors[i * 2].g = 0.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 0.0;
        edge_marker.colors[i * 2 + 1].g = 0.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        /*if(std::abs(v1->id() - v2->id()) > 2) {
          edge_marker.points[i * 2].z += 0.5;
          edge_marker.points[i * 2 + 1].z += 0.5;
        }*/

        continue;
      }

      
      g2o::EdgeSE2PriorXY* edge_priori_xy = dynamic_cast<g2o::EdgeSE2PriorXY*>(edge);
      if(edge_priori_xy) {
        
        g2o::VertexSE2* v1 = dynamic_cast<g2o::VertexSE2*>(edge_priori_xy->vertices()[0]);
        Eigen::Vector2d pt1 = v1->estimate().translation();
        Eigen::Vector2d pt2 = edge_priori_xy->measurement();

        
  

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = 0;
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = 0;

        edge_marker.colors[i * 2].r = 128.0/255.0;
        edge_marker.colors[i * 2].g = 128.0/255.0;
        edge_marker.colors[i * 2].b = 128.0/255.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 128.0/255.0;
        edge_marker.colors[i * 2 + 1].g = 128.0/255.0;
        edge_marker.colors[i * 2 + 1].b = 128.0/255.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }
      

      g2o::EdgeSE2Prior* edge_priori = dynamic_cast<g2o::EdgeSE2Prior*>(edge);
      if(edge_priori) {
        g2o::VertexSE2* v1 = dynamic_cast<g2o::VertexSE2*>(edge_priori->vertices()[0]);
        Eigen::Vector2d pt1 = v1->estimate().translation();
        Eigen::Isometry2d pt2 = edge_priori->measurement().toIsometry();

       
  

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        //edge_marker.points[i * 2].z = pt1.z() + 0.5;
        edge_marker.points[i * 2].z = 0;
        edge_marker.points[i * 2 + 1].x = pt2.translation().x();
        edge_marker.points[i * 2 + 1].y = pt2.translation().y();
        edge_marker.points[i * 2 + 1].z = 0;

        edge_marker.colors[i * 2].r = 255.0/255.0;
        edge_marker.colors[i * 2].g = 255.0/255.0;
        edge_marker.colors[i * 2].b = 0.0/255.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 255.0/255.0;
        edge_marker.colors[i * 2 + 1].g = 255.0/255.0;
        edge_marker.colors[i * 2 + 1].b = 0.0/255.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }

      g2o::EdgeSE2PointXY* edge_se2_xy = dynamic_cast<g2o::EdgeSE2PointXY*>(edge);
      if(edge_se2_xy) {

        g2o::VertexSE2* v1 = dynamic_cast<g2o::VertexSE2*>(edge_se2_xy->vertices()[0]);
        g2o::VertexPointXY* v2 = dynamic_cast<g2o::VertexPointXY*>(edge_se2_xy->vertices()[1]);
        Eigen::Vector2d pt1 = v1->estimate().translation();
        Eigen::Vector2d pt2 = v2->estimate();

        edge_marker.points[i * 2].x = pt1.x();
        edge_marker.points[i * 2].y = pt1.y();
        edge_marker.points[i * 2].z = 0;
        edge_marker.points[i * 2 + 1].x = pt2.x();
        edge_marker.points[i * 2 + 1].y = pt2.y();
        edge_marker.points[i * 2 + 1].z = 0;

        edge_marker.colors[i * 2].r = 255.0/255.0;
        edge_marker.colors[i * 2].g = 0.0/255.0;
        edge_marker.colors[i * 2].b = 255.0/255.0;
        edge_marker.colors[i * 2].a = 1.0;
        edge_marker.colors[i * 2 + 1].r = 255.0/255.0;
        edge_marker.colors[i * 2 + 1].g = 0.0/255.0;
        edge_marker.colors[i * 2 + 1].b = 255.0/255.0;
        edge_marker.colors[i * 2 + 1].a = 1.0;

        continue;
      }
    }

    // sphere
    visualization_msgs::Marker& sphere_marker = markers.markers[3];
    sphere_marker.header.frame_id = "map";
    sphere_marker.header.stamp = stamp;
    sphere_marker.ns = "loop_close_radius";
    sphere_marker.id = 3;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;

    if(!keyframes.empty()) {
      Eigen::Vector2d pos = keyframes.back()->node->estimate().translation();
      sphere_marker.pose.position.x = pos.x();
      sphere_marker.pose.position.y = pos.y();
      sphere_marker.pose.position.z = 0;
    }
    sphere_marker.pose.orientation.w = 1.0;
    sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = loop_detector->get_distance_thresh() * 2.0;

    sphere_marker.color.r = 1.0;
    sphere_marker.color.a = 0.3;
    return markers;
  }

  /**
   * @brief dump all data to the current directory
   * @param req
   * @param res
   * @return
   */
  bool dump_service(hdl_graph_slam::DumpGraphRequest& req, hdl_graph_slam::DumpGraphResponse& res) {
    std::lock_guard<std::mutex> lock(main_thread_mutex);

    std::string directory = req.destination;

    if(directory.empty()) {
      std::array<char, 64> buffer;
      buffer.fill(0);
      time_t rawtime;
      time(&rawtime);
      const auto timeinfo = localtime(&rawtime);
      strftime(buffer.data(), sizeof(buffer), "%d-%m-%Y %H:%M:%S", timeinfo);
    }

    if(!boost::filesystem::is_directory(directory)) {
      boost::filesystem::create_directory(directory);
    }

    std::cout << "all data dumped to:" << directory << std::endl;

    graph_slam->save(directory + "/graph.g2o");
    for(int i = 0; i < keyframes.size(); i++) {
      std::stringstream sst;
      sst << boost::format("%s/%06d") % directory % i;

      keyframes[i]->save(sst.str());
    }

    if(zero_utm) {
      std::ofstream zero_utm_ofs(directory + "/zero_utm");
      zero_utm_ofs << boost::format("%.6f %.6f %.6f") % zero_utm->x() % zero_utm->y() % 0.0 << std::endl;
    }

    std::ofstream ofs(directory + "/special_nodes.csv");
    ofs << "anchor_node " << (anchor_node == nullptr ? -1 : anchor_node->id()) << std::endl;
    ofs << "anchor_edge " << (anchor_edge == nullptr ? -1 : anchor_edge->id()) << std::endl;
    
    res.success = true;
    return true;
  }

  /**
   * @brief save map data as pcd
   * @param req
   * @param res
   * @return
   */
  bool save_map_service(hdl_graph_slam::SaveMapRequest& req, hdl_graph_slam::SaveMapResponse& res) {
    std::vector<KeyFrameSnapshot::Ptr> snapshot;

    keyframes_snapshot_mutex.lock();
    snapshot = keyframes_snapshot;
    keyframes_snapshot_mutex.unlock();

    auto cloud = map_cloud_generator->generate(snapshot, req.resolution);
    if(!cloud) {
      res.success = false;
      return true;
    }

    if(zero_utm && req.utm) {
      for(auto& pt : cloud->points) {
        Eigen::Vector3f zero_utm_3d = Eigen::Vector3f::Identity();
        zero_utm_3d.block<2,1>(0,0) = (*zero_utm).cast<float>();
        zero_utm_3d(2,0) = 0;
        pt.getVector3fMap() += zero_utm_3d;
      }
    }

    cloud->header.frame_id = map_frame_id;
    cloud->header.stamp = snapshot.back()->cloud->header.stamp;

    if(zero_utm) {
      std::ofstream ofs(req.destination + ".utm");
      ofs << (*zero_utm).transpose() << std::endl;
    }

    int ret = pcl::io::savePCDFileBinary(req.destination, *cloud);
    res.success = ret == 0;

    return true;
  }

private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;
  ros::WallTimer optimization_timer;
  ros::WallTimer map_publish_timer;

  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;
  std::unique_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync;

  ros::Subscriber gps_sub;
  ros::Subscriber nmea_sub;
  ros::Subscriber navsat_sub;

  ros::Subscriber imu_sub;

  ros::Publisher markers_pub;

  std::string map_frame_id;
  std::string odom_frame_id;

  std::mutex trans_odom2map_mutex;
  Eigen::Matrix3f trans_odom2map;
  ros::Publisher odom2map_pub;

  std::string points_topic;
  ros::Publisher read_until_pub;
  ros::Publisher map_points_pub;

  tf::TransformListener tf_listener;

  ros::ServiceServer dump_service_server;
  ros::ServiceServer save_map_service_server;

  // keyframe queue
  std::string base_frame_id;
  std::mutex keyframe_queue_mutex;
  std::deque<KeyFrame::Ptr> keyframe_queue;

  // gps queue
  double gps_time_offset;
  double gps_edge_stddev_xy;
  boost::optional<Eigen::Vector2d> zero_utm;
  std::mutex gps_queue_mutex;
  std::deque<geographic_msgs::GeoPointStampedConstPtr> gps_queue;

  // imu queue
  double imu_time_offset;
  bool enable_imu_orientation;
  double imu_orientation_edge_stddev;
  bool enable_imu_acceleration;
  double imu_acceleration_edge_stddev;
  std::mutex imu_queue_mutex;
  std::deque<sensor_msgs::ImuConstPtr> imu_queue;

  // for map cloud generation
  std::atomic_bool graph_updated;
  double map_cloud_resolution;
  std::mutex keyframes_snapshot_mutex;
  std::vector<KeyFrameSnapshot::Ptr> keyframes_snapshot;
  std::unique_ptr<MapCloudGenerator> map_cloud_generator;

  // graph slam
  // all the below members must be accessed after locking main_thread_mutex
  std::mutex main_thread_mutex;

  int max_keyframes_per_update;
  std::deque<KeyFrame::Ptr> new_keyframes;

  g2o::VertexSE2* anchor_node;
  g2o::EdgeSE2* anchor_edge;
  std::vector<KeyFrame::Ptr> keyframes;
  std::unordered_map<ros::Time, KeyFrame::Ptr, RosTimeHash> keyframe_hash;

  std::unique_ptr<GraphSLAM> graph_slam;
  std::unique_ptr<LoopDetector> loop_detector;
  std::unique_ptr<KeyframeUpdater> keyframe_updater;
  std::unique_ptr<NmeaSentenceParser> nmea_parser;

  std::unique_ptr<InformationMatrixCalculator> inf_calclator;

  // newly added buildings vars
  std::vector<BuildingNode::Ptr> buildings; // all buildings ever seen by the start of the system
  float lidar_range; // range used to download buildings
  int enter; // debug variable: true = download buildings, false = do not download buildings
  int zero_utm_zone; // utm zone of zero_utm
  char zero_utm_band; // utm band of zero_utm
  ros::Publisher buildings_pub; // publish cloud with buildings being currently aligned
  ros::Publisher odom_pub; // publish keyframe cloud being currently aligned
  ros::Publisher transformed_pub; // publish keyframe cloud used to perform alignment already aligned
  ros::Publisher original_odom_pub; // publish keyframe cloud before any filtering
  ros::Publisher estimated_buildings_pub; // publish cloud with estimated buildings
  ros::Publisher all_buildings_pub; // publish the openstreetmap buildings
  ros::Publisher temp_pub; // temporary publisher for debug
  double ground_floor_max_thresh; // threshold used to cut the floor
  double radius_search; // outlier removal radius parameter
  int min_neighbors_in_radius; // outlier removal parameter
  int ii; // order of arrival of keyframes: incremented every time a new keyframe is created
  std::vector<Eigen::Matrix4d> gt; // vector of ground truth poses
  ros::Publisher gt_markers_pub; // publisher of ground truth markers
  bool first_guess; // true if this is the first guess ever used since the start of the system, false otherwise
  // first_guess may be put back to true if the functions for dynamical tuning with sparse buildings bags are activated
  Eigen::Matrix4f prev_guess; // contains the last alignment transformation computed
  tf::TransformBroadcaster b_tf_broadcaster; // used to publish transforms
  bool fix_first_building; // fix the first building node ever inserted - not used anymore, may be removed
  bool reset_gicp; // used for dynamical tuning with sparse buildings bags
  int reset_counter; // used for dynamical tuning with sparse buildings bags
  double dist_last_kf; // used for dynamical tuning with sparse buildings bags
  Eigen::Vector3d zero_utm_lla; // used to publish zero_utm for the rviz_satellite package
  
  ros::Publisher zero_utm_pub; // publisher of zero_utm_lla
};

}  // namespace hdl_graph_slam

PLUGINLIB_EXPORT_CLASS(hdl_graph_slam::HdlGraphSlamNodelet, nodelet::Nodelet)
