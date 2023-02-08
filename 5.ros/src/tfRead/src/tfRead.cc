#include"ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/PointStamped.h"





int main(int argc,char ** argv)
{
    ros::init(argc,argv, "tf2_read");
    ros::NodeHandle nh;
    
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    ros::Rate rate(50);
 
     while(nh.ok())
     {
            geometry_msgs::TransformStamped transformStamped;
            try
            {
                transformStamped= buffer.lookupTransform("footprint","map",ros::Time(0));
            }
            catch(const std::exception& e)
            {
                    ROS_WARN("%s",e.what());
                    ros::Duration(1.0).sleep();
                    continue;
            }
            ROS_INFO("x:  %f, y: %f,    ",transformStamped.transform.translation.x,transformStamped.transform.translation.y);

            rate.sleep() ;  
     }
     return 0;
}