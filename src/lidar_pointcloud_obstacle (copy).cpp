#include <lidar_pointcloud_obstacle/lidar_pointcloud_obstacle.h>
#define MAP_IDX(sx, i, j) (sx * i + j)
#define PI 3.14159265
using namespace std;
using namespace cv;
namespace LPO
{
    // Default plugin constructor
    LidarPointcloudObstacle::LidarPointcloudObstacle()
    {
        ros::NodeHandle nPrivateHandle("~");
        nPrivateHandle.param<double>("elevation_resolution", resolution_z, 0.01); //[m]
        nPrivateHandle.param<double>("max_observable_distance", max_observable_distance, 10.0); //[m]
        nPrivateHandle.param<double>("min_observable_distance", min_observable_distance, -1.0); //[m]
        nPrivateHandle.param<int>("unknown_elevation", unknown_elevation, -100); //[cell]
        nPrivateHandle.param<double>("min_observable_elevation", min_elevation, 0.06); //[m]
        nPrivateHandle.param<double>("max_observable_elevation", max_elevation, 2.00); //[m]
        nPrivateHandle.param<double>("map_resolution", resolution_xy, 0.02); //[m]
        nPrivateHandle.param<int>("max_grid_size_x", width, 500); //[cell]
        width_half = width/2;
        nPrivateHandle.param<int>("max_grid_size_y", height, 500); //[cell]
        height_half = height/2;
        nPrivateHandle.param<int>("search_size", search_size, 20); //[cell]
        nPrivateHandle.param<int>("filter_radius", filter_radius, 2); //[cell]
        nPrivateHandle.param<int>("median_filter_size", median_filter_size, 1); //[cell]
        nPrivateHandle.param<int>("isolation_filter_threshold", isolation_filter_threshold, 3); //[cell]
        nPrivateHandle.param<int>("connection_filter_threshold", connection_filter_threshold, 8); //[cell]
        nPrivateHandle.param<double>("max_grad", max_grad, 10.0); //[cm]
        ROS_INFO("max_grad is %f",max_grad);
        nPrivateHandle.param<std::string>("local_map_frame_id", local_map_frame_id,std::string("odom"));
        nPrivateHandle.param<std::string>("baselink_frame_id", baselink_frame_id,std::string("base_link"));
        nPrivateHandle.param<std::string>("point_cloud_topic", point_cloud_topic, std::string("/rslidar_points"));
        elevation_data = std::vector<int16_t>(width * height,(int16_t)unknown_elevation);
        temp_data = std::vector<int16_t>(width * height,(int16_t)unknown_elevation);
        //accumulation_map = std::vector<int16_t>(width * height,(int16_t)unknown_elevation);
        sub_pointCloud = nPrivateHandle.subscribe(point_cloud_topic,1,&LidarPointcloudObstacle::lidarcloudCallback,this);
        sub_odometry = nPrivateHandle.subscribe("/odometry/filtered/local",1,&LidarPointcloudObstacle::odomCallback,this);
        pub_points = nPrivateHandle.advertise<sensor_msgs::PointCloud>("obstacle_points", 1, true);
        isFirstPC = true;
        isOdomRec = false;
        positionx = 0;
        positiony = 0;
        accumulation_map.clear();

       // pub_scan  = nPrivateHandle.advertise<sensor_msgs::PointCloud2>("obstacle_points2", 1, true);

//        *mapPtr = accumulation_map;
        ROS_INFO("P_O: is up and running.");
    }

//    int16_t LidarPointcloudObstacle::get_median_num(int16_t* num_array, int len)
//    {
//        int     i,j;            // 循环变量
//        int16_t num_temp;

//        // 用冒泡法对数组进行排序
//        for (j = 0; j < len - 1; j ++)
//        {
//            for (i = 0; i < len - j - 1; i ++)
//            {
//                if (num_array[i] > num_array[i + 1])
//                {
//                    // 互换
//                    num_temp = num_array[i];
//                    num_array[i] = num_array[i + 1];
//                    num_array[i + 1] = num_temp;
//                }
//            }
//        }

//        // 返回中间一个元素
//        num_temp = num_array[len / 2];

//        return num_temp;
//    }


    // Default deconstructor
    LidarPointcloudObstacle::~LidarPointcloudObstacle()
    {
        ROS_INFO("P_O: Shutting down!");

    }

    void  LidarPointcloudObstacle::odomCallback(const nav_msgs::OdometryConstPtr& odom_msg)
    {
        positionx = odom_msg->pose.pose.position.x;
        positiony = odom_msg->pose.pose.position.y;
        isOdomRec = true;

    }
    void LidarPointcloudObstacle::pubAccMap(std::vector<int16_t> accmap)
    {
        double mapsize = sizeof(accmap);
    }

    // cloudCallback get called if a new 3D point cloud is avaible
    /**
    * \param [in] pointcloud2_sensor_msg contains the 3D point cloud
    */
    void  LidarPointcloudObstacle::lidarcloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud2_sensor_msg)
    {
        if(!isOdomRec)
        {
            ROS_WARN("No odom info!");
            return;
        }

        if(isFirstPC)
        {
            originx = positionx;
            originy = positiony;
            isFirstPC = false;
        }

        //ROS_INFO("Received pointcloud.");
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud2_odom_pcl (new pcl::PointCloud<pcl::PointXYZ>()),
        pointcloud2_sensor_pcl (new pcl::PointCloud<pcl::PointXYZ> ()), baselink_pcl (new pcl::PointCloud<pcl::PointXYZ> ()), pass_pcl (new pcl::PointCloud<pcl::PointXYZ> ());

        pcl::fromROSMsg(*pointcloud2_sensor_msg, *pointcloud2_sensor_pcl);
        obstacle_points.points.clear();
        geometry_msgs::Point32 point;
//        pcl::PointXYZ point;
//        double accSize = accumulation_map.size();
//        double index=0;
//       // pcl::PointCloud<pcl::PointXYZ>::iterator Iter;

//        ROS_INFO("accSize is %f",accSize);
//        if(accSize > 10)
//        {
//            for(pcl::PointCloud<pcl::PointXYZ>::iterator Iter = accumulation_map.points.begin(); Iter != accumulation_map.points.end(); )
//            {

//                if(fabs(accumulation_map.points[index].x-positionx) > 7 || fabs(accumulation_map.points[index].y-positiony) > 7)
//                {
//                    Iter=accumulation_map.points.erase(Iter);
//                }
//                else
//                {
//                    Iter++;
//                    index++;
//                }
//            }
//        }


        // transform cloud to /odom frame
        try
        {
            listener.waitForTransform(local_map_frame_id, pointcloud2_sensor_msg->header.frame_id,pointcloud2_sensor_msg->header.stamp,ros::Duration(1.0));
            ROS_INFO("wait for transform");
            pcl_ros::transformPointCloud(local_map_frame_id,*pointcloud2_sensor_pcl,*pointcloud2_odom_pcl,listener);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            return;
        }
        ROS_INFO("transform ok!");
        unsigned int size = (unsigned int)pointcloud2_odom_pcl->points.size();
        if( size == pointcloud2_odom_pcl->width * pointcloud2_odom_pcl->height && size>0)
        {
            // iterate trough all points
            for (unsigned int k = 0; k < size; ++k)
            {
                const pcl::PointXYZ& pt_cloud = pointcloud2_odom_pcl->points[k];
                // check for invalid measurements
                if (isnan(pt_cloud.x) || isnan(pt_cloud.y) || isnan(pt_cloud.z))
                    continue;

                // check max distance
//                if( pointcloud2_sensor_pcl->points[k].y > max_observable_distance || pointcloud2_sensor_pcl->points[k].y < min_observable_distance || pointcloud2_sensor_pcl->points[k].x > max_observable_distance || pointcloud2_sensor_pcl->points[k].x < -max_observable_distance)
//                    continue;
//                if( pointcloud2_sensor_pcl->points[k].z > max_observable_distance || pointcloud2_sensor_pcl->points[k].z < -max_observable_distance)
//                    continue;
                double x_index = round(pt_cloud.x/resolution_xy);
                double y_index = round(pt_cloud.y/resolution_xy);
                int z_index = round(pt_cloud.z/resolution_xy);

                string accmap_key="";
                accmap_key += to_string(x_index);
                accmap_key += to_string(y_index);
                if(accumulation_map.count(accmap_key) > 0)
                {
                    if(z_index > accumulation_map[accmap_key]) accumulation_map[accmap_key] = z_index;
                }
                else
                {
                    accumulation_map.insert(map<string,int >::value_type(accmap_key,z_index));
                }

            }
            ROS_INFO("elevation ok!");

            for (int index_y = 0; index_y < width; ++index_y)
            {
                for (int index_x = 0; index_x < height; ++index_x)
                {
                    string index_key="";
                    index_key += to_string(index_x+round(positionx/resolution_xy)-height_half);
                    index_key += to_string(index_y+round(positiony/resolution_xy)-width_half);
                    if(accumulation_map.count(index_key) == 0) continue;
                    point.x = (index_x+round(positionx/resolution_xy)-height_half)*resolution_xy;
                    point.y = (index_y+round(positiony/resolution_xy)-width_half)*resolution_xy;
                    point.z = accumulation_map[index_key]*resolution_z;
                    obstacle_points.points.push_back(point);
                }
            }



            //发布points
//            pcl::toROSMsg(accumulation_map,obstacle_points);
            obstacle_points.header.frame_id = local_map_frame_id;
            obstacle_points.header.stamp = pointcloud2_sensor_msg->header.stamp;
            pub_points.publish(obstacle_points);

        }

    }
}
