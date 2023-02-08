#include "lesson2/scan_to_pointcloud2_converter.h"
#include <climits>


ScanToPointCloud2Converter::ScanToPointCloud2Converter():private_node_("~")
{
     ROS_INFO_STREAM("\033[1;32m----> Scan to PointCloud2 Converter.\033[0m");
     laser_scan_subscriber_ = node_handle_.subscribe("laser_scan", 1, &ScanToPointCloud2Converter::ScanCallback, this);

     // 注意，这里的发布器，发布的数据类型为 pcl::PointCloud<PointT>
    // ros中自动做了 pcl::PointCloud<PointT> 到 sensor_msgs/PointCloud2 的数据类型的转换.
    pointcloud2_publisher_ = node_handle_.advertise<PointCloudT>("pointcloud2_converted", 1, this);

    invalid_point_.x = std::numeric_limits<float>::quiet_NaN();
    invalid_point_.y = std::numeric_limits<float>::quiet_NaN();
    invalid_point_.z = std::numeric_limits<float>::quiet_NaN();
};


ScanToPointCloud2Converter::~ScanToPointCloud2Converter()
{
    ROS_INFO("Destroying ScanToPointCloud2Converter");
};


void ScanToPointCloud2Converter::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &msgs)
{
    PointCloudT::Ptr cloud_msg = boost::shared_ptr<PointCloudT>(new PointCloudT());

    cloud_msg->points.resize(msgs->ranges.size());

    for(unsigned int i=0; i <msgs->ranges.size();i++)
    {
        PointT & point_tmp = cloud_msg->points[i];
        float range = msgs->ranges[i];

          // 将 inf 与 nan 点 设置为无效点
        if(!std::isfinite(range))
        {
             std::cout << " " << i << " " << msgs->ranges[i];
            point_tmp = invalid_point_;
            continue;
        }
        // 有些雷达驱动会将无效点设置成 range_max+1
        // 所以要根据雷达的range_min与range_max进行有效值的判断

        if(range>msgs->range_min && range<msgs->range_max)
        {
            float angle =  msgs->angle_min +i* msgs->angle_increment;

            point_tmp.x = range*cos(angle);
            point_tmp.y = range*sin(angle);
            point_tmp.z = 0.0;
        }
        else
        {
            point_tmp = invalid_point_;
        }
        cloud_msg->width  = msgs->ranges.size();
        cloud_msg->height = 1;
        cloud_msg->is_dense = false;

        //将msgs 的消息头赋值到PointCloudT的消息头
        pcl_conversions::toPCL(msgs->header,cloud_msg->header);
        pointcloud2_publisher_.publish(cloud_msg);



    }
}
