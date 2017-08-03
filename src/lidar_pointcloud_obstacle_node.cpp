#include <lidar_pointcloud_obstacle/lidar_pointcloud_obstacle.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_pointcloud_obstacle_node");

  // Shared parameters to be propagated to nodelet private namespaces
  LPO::LidarPointcloudObstacle lidarpointclouds;

  ros::spin();
  return 0;
}
