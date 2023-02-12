#include "lesson2/scan_match_icp.h"
#include<chrono>

ScanMatchICP::ScanMatchICP()
{
     ROS_INFO_STREAM("\033[1;32m----> Scan Match with ICP started.\033[0m");
     laser_scan_subscriber_ = node_handle_.subscribe("laser_scan",1,&ScanMatchICP::ScanCallback,this);
     is_first_scan_ = true;
    current_pointcloud_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    last_pointcloud_ = boost::shared_ptr<PointCloudT>(new PointCloudT());

}
ScanMatchICP::~ScanMatchICP()
{
    ROS_INFO("Destroying ScanMatchICP");
}

void ScanMatchICP::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msgs)
{
    std::chrono::steady_clock::time_point   start_time = std::chrono::steady_clock::now();

    if(is_first_scan_ )
    {
        ConvertScan2PointCloud(scan_msgs);
        is_first_scan_ = false;
        return;
    }
    else
        *last_pointcloud_ = *current_pointcloud_;
     ConvertScan2PointCloud(scan_msgs);
    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time-start_time);
     std::cout << "\n转换数据格式用时: " << time_used.count() << " 秒。" << std::endl;

     
    start_time = std::chrono::steady_clock::now();
    ScanMatchWithICP(scan_msgs);

    end_time =std::chrono::steady_clock::now();

    time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);

   std::cout << "ICP计算用时: " << time_used.count() << " 秒。" << std::endl;

}

void ScanMatchICP::ConvertScan2PointCloud(const sensor_msgs::LaserScan::ConstPtr&scan_msg)
{
    PointCloudT::Ptr cloud_msg = boost::shared_ptr<PointCloudT>(new PointCloudT());

    cloud_msg->points.resize(scan_msg->ranges.size());

    for(int i=0; i<scan_msg->ranges.size();i++)
    {
        //这位置一定要拿引用
        pcl::PointXYZ& point_tmp = cloud_msg->points[i];
        
        float range = scan_msg->ranges[i];
        if(!std::isfinite(range))
            continue;
        if(range>scan_msg->range_min && range<scan_msg->range_max)
        {
            float angle  =scan_msg->angle_min + i* scan_msg->angle_max;
            point_tmp.x = range * cos(angle);
            point_tmp.y = range *sin(angle);
            point_tmp.z = 0.0;
        }

        cloud_msg->width = scan_msg->ranges.size();
        cloud_msg->height = 1;
        cloud_msg->is_dense =  true;
        pcl_conversions::toPCL(scan_msg->header,cloud_msg->header);

        *current_pointcloud_ = *cloud_msg;
    }
}

void::ScanMatchICP::ScanMatchWithICP(const sensor_msgs::LaserScan::ConstPtr&scan_msg)
{
      icp_.setInputSource(last_pointcloud_);
    icp_.setInputTarget(current_pointcloud_);

    pcl::PointCloud<pcl::PointXYZ>unused_result;
    icp_.align(unused_result);

    if(!icp_.hasConverged())
    {
         std::cout << "not Converged" << std::endl;
        return;
    }
    Eigen::Affine3f transform;
    transform = icp_.getFinalTransformation();

    float x ,y ,z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transform,x,y,z,roll,pitch,yaw);
     std::cout << "transfrom: (" << x << ", " << y << ", " << yaw * 180 / M_PI << ")" << std::endl;

}




