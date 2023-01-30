#include<iostream>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "Eigen/Core"
#include<vector>
#include"pcl/visualization/pcl_visualizer.h"
#include "pcl/features/boundary.h"
#include"pcl/kdtree/kdtree.h"
#include"pcl/features/normal_3d.h"


typedef pcl::PointXYZ  pointT;
int main(int argc, char** argv)
{
    pcl::PointCloud<pointT>::Ptr cloud(new pcl::PointCloud<pointT>);
    pcl::PCDReader reader;
     reader.read("../milk.pcd",*cloud);
    //法线估计
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pointT, pcl::Normal>normalEstimation;
    normalEstimation.setInputCloud(cloud);
    normalEstimation.setRadiusSearch(0.03);
    pcl::search::KdTree<pointT>::Ptr kdtree(new pcl::search::KdTree<pointT>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);


    //边界估计
    pcl::BoundaryEstimation<pointT,pcl::Normal,pcl::Boundary>est;
    pcl::PointCloud<pcl::Boundary> boundaries;  
    est.setInputCloud(cloud);
    est.setInputNormals(normals);
    est.setRadiusSearch(0.02);
    est.setSearchMethod(typename pcl::search::KdTree<pcl::PointXYZ>::Ptr (new pcl::search::KdTree<pcl::PointXYZ>));
    est.compute(boundaries);

    std::cout<<cloud->size()<<std::endl;
    std::cout<<boundaries.size()<<std::endl;


//可视化
    pcl::visualization::PCLVisualizer  viewer;
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0); // green
    viewer.addPointCloud(cloud, single_color);
	viewer.setBackgroundColor(0, 0, 0);

    for(int i=0; i<boundaries.size();i++)
    {
        if(boundaries.points[i].boundary_point>0)
        {
              pcl::PointXYZ c(cloud->points[i]);
              viewer.addSphere(c,0.005,1,12,0,"spherer"+i,0);
    
        }
    }



     while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
	return 0;
 



}