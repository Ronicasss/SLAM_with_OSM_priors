# SLAM_with_OSM_priors
To make the system work you need to install the packages and libraries required by [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam), on which the system is based.

Additionally, the [curlpp](http://www.curlpp.org/) library and the [rviz_satellite](https://github.com/nobleo/rviz_satellite) package are required.

The system works on Ubuntu with ROS Melodic.

To run it you need to open four terminals. Then do the following operations:
1) run "roscore" on terminal 1
2) run "rosparam set use_sim_time true" on terminal 2 
3) then run "roslaunch hdl_graph_slam kitti.launch" on terminal 2
4) navigate on the folder where there are bags and run "rosbag play --clock -r0.1 bag_name.bag" on terminal 3
5) navigate to the "rviz" folder of the package and run "rviz -d hdl_graph_slam.rviz" on terminal 4