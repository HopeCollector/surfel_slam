#include "ioVis.h"

pcl::PointCloud<pcl::PointXYZI>::Ptr getFile(std::string fileName){
    fstream file(fileName, ios::binary | ios::in);

    if(!file.good()){
        cerr << "Cannot open file" << fileName << endl;
        exit(EXIT_FAILURE);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    file.seekg(0, ios::beg);
    for(int i = 0; file.good() && !file.eof(); i++){
        pcl::PointXYZI point;
        file.read((char*)&point.x, 3*sizeof(float));
        file.read((char*)&point.intensity, sizeof(float));
        cloud -> push_back(point);
    }

    return cloud;
}

pcl::visualization::PCLVisualizer::Ptr normalVis(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normal){
    pcl::visualization::PCLVisualizer::Ptr vis(new pcl::visualization::PCLVisualizer);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler ("intensity");
    handler.setInputCloud(cloud);

    vis->setBackgroundColor(0,0,0);
    vis->addPointCloud<pcl::PointXYZI> (cloud, handler, "t");
    vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "t");
    vis->addPointCloudNormals<pcl::PointXYZI, pcl::Normal> (cloud, cloud_normal, 10, 0.05, "normals");
    vis->addCoordinateSystem (1.0);
    vis->initCameraParameters ();

    return vis;
}