#include <ros/ros.h>
#include <cmath>
#include <map>
#include <iostream>
#include <sstream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>

#include <std_msgs/String.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <stdio.h>
#include <pcl/io/pcd_io.h>

#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/random_sample.h>

#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/exceptions.h>
using namespace std;
using namespace cv;
namespace LPO{

    class LidarPointcloudObstacle
    {

    private:

        int16_t get_median_num(int16_t* num_array, int len);

        tf::TransformListener listener;
        tf::Transform Tftran;




        ros::Timer timer;


        double max_observable_distance;
        double min_observable_distance;
        double max_grad;//梯度差阈值，单位cm
        std::vector<int16_t> elevation_data;//高度地图
        std::vector<int16_t> accumulation_map;//高度地图
        std::vector<int16_t> temp_data;//高度地图缓存
        int **elevation_map;
        int **temp_map;
        int **init_map;
        int width;//地图的宽，单位m
        int width_half;
        int height;//地图的长，单位m
        int height_half;
        int unknown_elevation;
        double resolution_z;//z分辨率,单位m
        double resolution_xy;//xy分辨率,单位m
        double min_elevation;//最小高度
        double max_elevation;//最大高度
        int median_filter_size;
        int isolation_filter_threshold;
        int connection_filter_threshold;
        int filter_radius;
        int search_size;
        double positionx,positiony,originx,originy;
        sensor_msgs::PointCloud obstacle_points;
        bool isFirstPC,isOdomRec;
        //pcl::SACSegmentation<pcl::PointXYZ>  seg;


        std::string local_map_frame_id;
        std::string baselink_frame_id;
        std::string point_cloud_topic;

        ros::Subscriber sub_pointCloud;
        ros::Subscriber sub_odometry;

        ros::Publisher pub_points;
       // ros::Publisher pub_scan;

        pcl::PointCloud<pcl::PointXYZ> accumulation_points;

    public:
        /// Default plugin constructor

        LidarPointcloudObstacle();

        /// Default deconstructor
        ~LidarPointcloudObstacle();

        /// cloudCallback get called if a new 3D point cloud is avaible
        /**
        * \param [in] pointcloud2_sensor_msg contains the 3D point cloud
        */
        void lidarcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud2_sensor_msg);
        void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);
        void pubAccMap(std::vector<int16_t> accmap);

    };
}
