#ifndef   LESSON2_SCAN_MATCH_ICP
#define  LESSON2_SCAN_MATCH_ICP

//ros
#include "ros/ros.h"
#include"sensor_msgs/LaserScan.h"

//pcl_ros
#include "pcl_ros/point_cloud.h"

//pcl
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"


class ScanMatchICP
{
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
private:
    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_;
    ros::Subscriber laser_scan_subscriber_;

    bool is_first_scan_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_pointcloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr last_pointcloud_;

    //icp 
    pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp_;

    void ScanMatchWithICP(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void ConvertScan2PointCloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
public:
     ScanMatchICP();
    ~ScanMatchICP();
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
};




#endif //LESSON2_SCAN_MATCH_ICP