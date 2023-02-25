# SLAM_with_OSM_priors
This is a ROS package that implements a system for performing SLAM using OpenStreetMap priors.
To make the package work you need to install the packages and libraries required by [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam), on which the system is based.

Additionally, the [curlpp](http://www.curlpp.org/) library and the [rviz_satellite](https://github.com/nobleo/rviz_satellite) packages are required.

The package has been tested on Ubuntu with ROS Melodic.

To run it you need to open four terminals. Then do the following operations:
1) run "roscore" on terminal 1
2) run "rosparam set use_sim_time true" on terminal 2 
3) then run "roslaunch hdl_graph_slam kitti.launch" on terminal 2
4) navigate on the folder where there are bags and run "rosbag play --clock -r0.1 bag_name.bag" on terminal 3
5) navigate to the "rviz" folder of the package and run "rviz -d hdl_graph_slam.rviz" on terminal 4

if commands don't work try to source the terminals (especially required by terminal 2 for roslaunch and terminal 4 for rviz).

Bags are created with the tool [kitti2bag](https://github.com/tomas789/kitti2bag) from [kitti raw sequences](http://www.cvlibs.net/datasets/kitti/raw_data.php) synced. I had to modify the tool to remove the publication of the tf map -> base_link

2023-02-25 Thesis pdf was added
N.B.: This code is highly messy and chaotic. I plan to refactor it entirely
