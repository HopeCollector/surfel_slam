// TODO: 切分点云数据
// TODO: 从单块点云数据中提取特征


#include "ioVis.h"
#include <pcl/features/normal_3d.h>

pcl::PointCloud<pcl::Normal>::Ptr genSurfelCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr);

int main(int argc, char* argv[]){
    
    if(argc != 2){
        std::cerr << "Need input file" << endl;
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = getFile(argv[1]);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal = genSurfelCloud(cloud);
    pcl::visualization::PCLVisualizer::Ptr visualizer = normalVis(cloud, cloud_normal);

    while (!visualizer->wasStopped())
    {
        visualizer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds (100000));
    }
    
    return 0;
}

pcl::PointCloud<pcl::Normal>::Ptr genSurfelCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);

    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.compute(*cloud_normal);

    return cloud_normal;
}