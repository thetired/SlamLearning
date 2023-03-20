#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include<iostream>
using namespace std;
int main( int argc, char** argv )
{
    ros::init(argc, argv, "showline");
    ros::NodeHandle n;
    ros::Publisher markerPub = n.advertise<visualization_msgs::Marker>("TEXT_VIEW_FACING", 10);
    visualization_msgs::Marker marker;
    marker.header.frame_id="/odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id =0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    marker.scale.z = 0.2;
    marker.color.b = 0;
    marker.color.g = 0;
    marker.color.r = 255;
    marker.color.a = 1;

    ros::Rate r(1);
    int k=0;
    while(1)
    {
        geometry_msgs::Pose pose;
        pose.position.x =  (float)(k++)/10;
        pose.position.y =  0;
        pose.position.z =0;
        ostringstream str;
        str<<k;
        marker.text=str.str();
        marker.pose=pose;
        markerPub.publish(marker);
        cout<<"k="<<k<<endl;
        r.sleep();
    }
    return 0;
}