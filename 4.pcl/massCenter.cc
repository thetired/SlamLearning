#include<iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "Eigen/Core"
#include"pcl/common/centroid.h"
#include<vector>
#include"pcl/visualization/pcl_visualizer.h"


typedef pcl::PointXYZ  pointT;
int main(int argc, char** argv)
{
    pcl::PointCloud<pointT>::Ptr cloud(new pcl::PointCloud<pointT>);

    pcl::PCDReader reader;
    reader.read("../milk.pcd",*cloud);

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud,centroid);

    std::vector<double> center(3,0.0);

    for(auto p :cloud->points)
    {
        center[0]+=p.x;
        center[1]+=p.y;
        center[2]+=p.z;
    }
    std::for_each(center.begin(),center.end(),[=](double& d){d/=cloud->points.size();});

    for(double m :center)
    {
        std::cout<<m<<" ";
    }
    std::cout<<"\n";

    std::cout<<centroid.transpose()<<std::endl;


    // 可视化
    pcl::visualization::PCLVisualizer  viewer;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0); // green
    viewer.addPointCloud(cloud, single_color);
	viewer.setBackgroundColor(0, 0, 0);

    pcl::PointXYZ c(center[0],center[1],center[2]);
    viewer.addSphere(c,0.005,1,12,0,"spherer",0);

    while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
	return 0;






    


    //std::cout<<cloud->size()<<std::endl;

}