#include <iostream>
#include <fstream>
#include <thread>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr getFile(std::string);
pcl::visualization::PCLVisualizer::Ptr normalVis(pcl::PointCloud<pcl::PointXYZI>::ConstPtr, pcl::PointCloud<pcl::Normal>::Ptr);
