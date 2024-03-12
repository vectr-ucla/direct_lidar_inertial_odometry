/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "dlio/dlio.h"

class dlio::MapNode {

public:

  MapNode(ros::NodeHandle node_handle);
  ~MapNode();

  void start();

private:

  void getParams();

  void callbackKeyframe(const sensor_msgs::PointCloud2ConstPtr& keyframe);

  bool savePcd(direct_lidar_inertial_odometry::save_pcd::Request& req,
               direct_lidar_inertial_odometry::save_pcd::Response& res);

  ros::NodeHandle nh;

  ros::Subscriber keyframe_sub;
  ros::Publisher map_pub;

  ros::ServiceServer save_pcd_srv;

  pcl::PointCloud<PointType>::Ptr dlio_map;
  pcl::VoxelGrid<PointType> voxelgrid;

  std::string odom_frame;

  double leaf_size_;

};
