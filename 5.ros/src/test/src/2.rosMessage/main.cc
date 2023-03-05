#include <ros/ros.h>
#include <ros/console.h>

int main( int argc, char **argv )
{
  ros::init( argc, argv, "rosdebug" );
  ros::NodeHandle n;
  ros::Rate rate( 1 );

  while( ros::ok() ) {
    ROS_INFO_STREAM_ONCE("loop start");
    ROS_DEBUG_STREAM( "DEBUG message." );
    ROS_INFO_STREAM ( "INFO message."<<666  );
    ROS_WARN_STREAM ( "WARN message."  );
    ROS_ERROR_STREAM( "ERROR message." );
    ROS_FATAL_STREAM( "FATAL message." );
    ROS_INFO_STREAM_NAMED( "named_msg", "INFO named message." );
    ROS_INFO_STREAM_THROTTLE( 2, "INFO throttle message." );
    ros::spinOnce();
    rate.sleep();
  }
  return 1;
}
