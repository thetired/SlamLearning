#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl_conversions/pcl_conversions.h>
#include<sensor_msgs/PointCloud2.h>

int main(int argc, char** argv)
{
    ros::init(argc,argv,"pcl_create");

    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_output",1);
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    cloud.width = 100;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);

    // 生成随机点云
    for(int i=0; i<cloud.points.size();i++)
    {
        cloud.points[i].x = 1024*rand()/(RAND_MAX+1.0f);
        cloud.points[i].y = 1024*rand()/(RAND_MAX+1.0f);
        cloud.points[i].z = 1024*rand()/(RAND_MAX+1.0f);
    }
    //convert to ros message
    pcl::toROSMsg(cloud,output);
    output.header.frame_id = "odom";

    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;


    //fill in the cloud data;

}