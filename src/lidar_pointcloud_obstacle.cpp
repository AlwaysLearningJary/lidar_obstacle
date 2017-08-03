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
        nPrivateHandle.param<int>("unknown_elevation", unknown_elevation, 50); //[cell]
        nPrivateHandle.param<double>("min_observable_elevation", min_elevation, 0.06); //[m]
        nPrivateHandle.param<double>("max_observable_elevation", max_elevation, 2.00); //[m]
        nPrivateHandle.param<double>("map_resolution", resolution_xy, 0.02); //[m]
        nPrivateHandle.param<int>("max_grid_size_x", width, 1200); //[cell]
        width_half = width/2;
        nPrivateHandle.param<int>("max_grid_size_y", height, 1200); //[cell]
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
        accumulation_map = std::vector<int16_t>(width * height,(int16_t)unknown_elevation);
        sub_pointCloud = nPrivateHandle.subscribe(point_cloud_topic,1,&LidarPointcloudObstacle::lidarcloudCallback,this);
        sub_odometry = nPrivateHandle.subscribe("/odometry/filtered/local",1,&LidarPointcloudObstacle::odomCallback,this);
        pub_points = nPrivateHandle.advertise<sensor_msgs::PointCloud>("obstacle_points", 1, true);
        isFirstPC = true;
        isOdomRec = false;
        positionx = 0;
        positiony = 0;
        elevation_map = new int *[height];
        temp_map = new int *[height];
        init_map = new int *[height];


        for(int i=0; i<height; i++)
        {
            elevation_map[i] =new int [width];
            temp_map[i] =new int [width];
            init_map[i] = new int [width];
            for(int j=0;j<width;j++)
            {
                elevation_map[i][j] = unknown_elevation;
                temp_map[i][j] = unknown_elevation;
                init_map[i][j] = unknown_elevation;
            }

        }
//        cout << "" << elevation_map[22][12] << endl;



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
//        cout << "verti " << elevation_map[22][12] << endl;
        if(fabs(positionx-originx) > 1)
        {
         /*   temp_map = elevation_map;
            elevation_map.clear();
            double relative_x = round((positionx - originx)/resolution_xy);
            double relative_y = round((positiony - originy)/resolution_xy);
            map<string,int>::iterator iter;
            for(iter = temp_map.begin(); iter != temp_map.end(); iter++)
            {
                string key_temp = iter->first;
                int value_temp = iter->second;
                double temp_x=0,temp_y=0;
                char *p=NULL;
                char *key_p = const_cast<char*>(key_temp.c_str());

                const char *pDelimiter = "/";
                p = strtok(key_p , pDelimiter);
                cout << "keytemp p1 = " << p << endl;
                temp_x = atof(p);
                cout << "temp_x = " << temp_x << endl;
                p = strtok(NULL , pDelimiter);
                cout << "keytemp p2 = " << p << endl;
                temp_y = atof(p);
                cout << "temp_y = " << temp_y << endl;

                temp_x -= relative_x;
                temp_y -= relative_y;
                if(fabs(temp_x) > height_half || fabs(temp_y) > width_half) continue;
                key_temp="";
                key_temp += to_string(temp_x);
                key_temp += "/";
                key_temp += to_string(temp_y);

                elevation_map.insert(map<string,int >::value_type(key_temp,value_temp));

            }

//            elevation_data = std::vector<int16_t>(width * height, (int16_t)unknown_elevation);//创建空地图vector
//            int relative_index = MAP_IDX(width,
//              (int)round((positionx-originx)/resolution_xy)+height_half,
//              (int)round((positiony-originy)/resolution_xy)+width_half);
//            originx = positionx;
//            originy = positiony;
//            for(int i = 0; i < width * height; i++)
//            {
//                if(i+relative_index >= width * height || i+relative_index < 0) continue;
//                elevation_data[i] = temp_data[i+relative_index];
//            }*/

 //           cout << "elevation_map size is : " << sizeof(elevation_map);//sizeof返回的是指针的大小，只有1个字节8位
            for(int ii = 0; ii < height; ii++)
            {
                memcpy(temp_map[ii], elevation_map[ii], sizeof(int)*width);
                memcpy(elevation_map[ii], init_map[ii], sizeof(int)*width);

            }
//            memcpy(temp_map, elevation_map, sizeof(int)*width*height);
//            memset(elevation_map, unknown_elevation, sizeof(int)*width*height);
            int relative_x = (int)round((positionx - originx)/resolution_xy);
            int relative_y = (int)round((positiony - originy)/resolution_xy);
            originx = positionx;
            originy = positiony;
            for(int i = 0; i < height; i++)
            {
                for(int j = 0; j < width; j++)
                {
                    if((i+relative_x) < 0 || (i+relative_x) >= height || (j+relative_y) < 0 || (j+relative_y) >= width) continue;
                    elevation_map[i][j]=temp_map[i+relative_x][j+relative_y];
                }
            }
        }

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


//        ROS_INFO("transform ok!");
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

//                if(fabs(round((pt_cloud.x-originx)/resolution_xy)) >= height_half) continue;
//                if(fabs(round((pt_cloud.y-originy)/resolution_xy)) >= width_half) continue;
//                int cell_index = MAP_IDX(width,
//                  (int)round((pt_cloud.x-originx)/resolution_xy)+height_half,
//                  (int)round((pt_cloud.y-originy)/resolution_xy)+width_half);
//                if(cell_index < 0 || cell_index >= height * width) continue;
                int x_index = (int)round((pt_cloud.x-originx)/resolution_xy) + height_half;
                int y_index = (int)round((pt_cloud.y-originy)/resolution_xy) + width_half;
                int z_index = (int)round(pt_cloud.z/resolution_z);
//                cout << "pt.x is " << pt_cloud.x << " ;  x_index is " << x_index << "  ; originx  " << originx <<endl;

                if(x_index >= height || y_index >= width || x_index < 0 || y_index < 0) continue;
//                cout << "z_index " << z_index << endl;
//                cout << "elevation_map " << elevation_map[x_index][y_index] << endl;
                int ele_value = elevation_map[x_index][y_index];
                if(z_index < ele_value) elevation_map[x_index][y_index] = z_index;
//                cout << "pt.x is " << pt_cloud.x << " ;  pt.y is " << pt_cloud.y << endl;
//                cout << "the elevation_map[" << x_index << "][" << y_index << "] is : " << elevation_map[x_index][y_index] << endl;
//                string accmap_key="";
//                accmap_key += to_string(x_index);
//                accmap_key += "/";
//                accmap_key += to_string(y_index);
////                cout << "accmap_key = " << accmap_key << endl;
//                if(elevation_map.count(accmap_key) > 0)
//                {
//                    if(z_index < elevation_map[accmap_key]) elevation_map[accmap_key] = z_index;
//                }
//                else
//                {
//                    elevation_map.insert(map<string,int >::value_type(accmap_key,z_index));
//                }

//                int16_t* pt_local_map = &elevation_data[cell_index];

//                // elevation in current cell in meter
//                double cell_elevation = resolution_z*(*pt_local_map);
//                // store maximum of each cell
//                if(pt_cloud.z < cell_elevation)
//                {
//                    *pt_local_map = (int16_t)round(pt_cloud.z/resolution_z);
//                }

            }

            for (int j = 0; j < width; j++)
            {
                for (int i = 0; i < height; i++)
                {
                    if(elevation_map[i][j] == unknown_elevation)
                        continue;
                    point.x = (i-height_half) * resolution_xy + originx;
                    point.y = (j-width_half) * resolution_xy + originy;
                    int valuez = elevation_map[i][j];
                    point.z = valuez * resolution_z;

//                    cout << "pt.x is " << point.x << " ;  pt.y is " << point.y << "  ;  pt.z  " << point.z << endl;
                   // cout << "originx" << originx <<endl;
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
