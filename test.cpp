#include "ioVis.h"
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>

int main(){

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    //------------------------------------
    //-----Add shapes at cloud points-----
    //------------------------------------
    pcl::ModelCoefficients coeffs;
    // coeffs.values.push_back (0.0);
    // coeffs.values.push_back (0.0);
    // coeffs.values.push_back (1.0);
    // coeffs.values.push_back (0.0);
    // viewer->addPlane (coeffs, "plane");
    // coeffs.values.clear ();
    coeffs.values.push_back (0.3);
    coeffs.values.push_back (0.3);
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (0.5);
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (5.0);
    viewer->addCone (coeffs, "cone");

    coeffs.values.clear();
    coeffs.values.push_back (0.3);
    coeffs.values.push_back (1.0);
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (0.25);
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (0.0);
    coeffs.values.push_back (50.0);
    viewer->addCone (coeffs, "cone2");

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds (100000));
    }
    return 0;
}