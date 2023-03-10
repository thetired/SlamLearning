#include "ros/ros.h"
#include"sensor_msgs/LaserScan.h"


//声明一个类

class LaserScan
{
    private:
        ros::NodeHandle node_handle_ ;
        ros::NodeHandle private_node_;
        ros::Subscriber laser_scan_subscriber_;

    public:
        LaserScan();
        LaserScan(const LaserScan&) =delete;
        LaserScan& operator=(const LaserScan&) = delete;
        ~LaserScan(){};
        void  ScanCallback(const sensor_msgs::LaserScan::ConstPtr & scan_msg);
};

//选择在构造函数中初始化 subscriber 
LaserScan::LaserScan()
{
    ROS_INFO_STREAM("LaserScan initial .");
    laser_scan_subscriber_ = node_handle_.subscribe("laser_scan",1,&LaserScan::ScanCallback,this);
}

void LaserScan::ScanCallback(const sensor_msgs::LaserScan::ConstPtr & scan_msg)
{
    ROS_INFO_STREAM(
        "seqence: " << scan_msg->header.seq << 
        ", time stamp: " << scan_msg->header.stamp << 
        ", frame_id: " << scan_msg->header.frame_id << 
        ", angle_min: " << scan_msg->angle_min << 
        ", angle_max: " << scan_msg->angle_max << 
        ", angle_increment: " << scan_msg->angle_increment << 
        ", time_increment: " << scan_msg->time_increment << 
        ", scan_time: " << scan_msg->scan_time << 
        ", range_min: " << scan_msg->range_min << 
        ", range_max: " << scan_msg->range_max << 
        ", range size: " << scan_msg->ranges.size() << 
        ", intensities size: " << scan_msg->intensities.size());

        double range = scan_msg->ranges[4];
        double angle = scan_msg->angle_min + scan_msg->angle_increment*4;
        double x = range*cos(angle);
        double y = range*sin(angle);
        
         ROS_INFO_STREAM(
        // 第5个数据点对应的极坐标为: 
        "range = " << range << ", angle = " << angle << 
        // 第5个数据点对应的欧式坐标为: 
        ", x = " << x << ", y = " << y);


        //遍历雷达数据
        for(int i=0; i<scan_msg->ranges.size();i++)
        {}
    

}


int main(int argc, char**argv)
{
    ros::init(argc,argv,"lesson1_laser_scan_node");
    LaserScan laser_scan;

    ros::spin();
    return 0;
   

}