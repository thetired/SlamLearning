#include<iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include<pcl/io/pcd_io.h>
#include"pcl/filters/statistical_outlier_removal.h"


int main(int argc, char**argv)
{

//init
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  if(pcl::io::loadPCDFile<pcl::PointXYZ>("../ism_train_horse.pcd",*cloud)==-1)
            PCL_ERROR("FAIL TO OPENED FILED");

    std::cout<<cloud->size()<<std::endl;

    // 直通滤波
    pcl::PassThrough<pcl::PointXYZ>pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.1, 0.01);   // 在这个范围内被留下
    pass.setNegative(true); 
    pass.filter(*cloud_filtered);

    std::cout<<cloud_filtered->size()<<std::endl;

    //统计滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new::pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setMeanK(10);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_final);
    std::cout<<cloud_final->size()<<std::endl;

    //save
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("result.pcd",*cloud_final);





}