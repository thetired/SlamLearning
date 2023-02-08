#include "lesson2/scan_to_pointcloud2_converter.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"lesson2_scan_to_cloud_converter_node");
    ScanToPointCloud2Converter scan_to_cloud_converter;
    ros::spin();
    return 0;
}