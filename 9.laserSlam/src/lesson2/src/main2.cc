#include "lesson2/scan_match_icp.h"


int main(int argc, char** argv)
{
    ros::init(argc,argv,"lesson2_scan_match_icp_node");
     ScanMatchICP scan_match_icp;

     ros::spin();

     return 0;


}