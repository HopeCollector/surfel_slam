#include "ioVis.h"
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>

#define Pos(x,y,z) (x + y*xBlockNum + z*xBlockNum*yBlockNum)

pcl::PointCloud<pcl::PointXYZI>::Ptr genCloud(std::vector<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZINormal>::Ptr genSurfel(std::vector<pcl::PointXYZI>);

int main(int argc, char* argv[]){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = getFile(argv[1]);
    std::vector<pcl::PointXYZ> points;
    const float gridLength = 1.0;
    float max_x,min_x = 0;
    float max_y,min_y = 0;
    float max_z,min_z = 0;

    for (int i=0; i < cloud->size(); i++)
    {
        pcl::PointXYZ point;
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;
        points.push_back(point);

        max_x = point.x > max_x ? point.x : max_x;
        min_x = point.x < min_x ? point.x : min_x;

        max_y = point.y > max_y ? point.y : max_y;
        min_y = point.y < min_y ? point.y : min_y;

        max_z = point.z > max_z ? point.z : max_z;
        min_z = point.z < min_z ? point.z : min_z;
    }

    cout << "x 边界 " <<max_x << " " << min_x << endl;
    cout << "y 边界 " <<max_y << " " << min_y << endl;
    cout << "z 边界 " <<max_z << " " << min_z << endl;

    const int xBlockNum = (max_x - min_x) / gridLength;
    const int yBlockNum = (max_y - min_y) / gridLength;
    const int zBlockNum = (max_z - min_z) / gridLength;
    std::vector<pcl::PointXYZI> subClouds[xBlockNum+1][yBlockNum+1][zBlockNum+1];

    for(int i = 0; i < cloud->size(); i++){
        pcl::PointXYZI point = cloud->points[i];

        const int xPos = (point.x - min_x) / gridLength;
        const int yPos = (point.y - min_y) / gridLength;
        const int zPos = (point.z - min_z) / gridLength;

        subClouds[xPos][yPos][zPos].push_back(point);
    }


}

pcl::PointCloud<pcl::PointXYZI>::Ptr genCloud(std::vector<pcl::PointXYZI> points){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for(std::vector<pcl::PointXYZI>::iterator point = points.begin(); point != points.end(); point ++){
        cloud->push_back(*point);
    }
    
    return cloud;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr genSurfel(std::vector<pcl::PointXYZI> points){
    if(points.size() == 0){
        return nullptr;
    }
    
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    pcl::PointXYZINormal centerPoint;
    float cov[9] = {0.0};
    float v[9] = {0.0};


    for(std::vector<pcl::PointXYZI>::iterator point = points.begin(); point != points.end(); point ++){
        centerPoint.x += point->x;
        centerPoint.y += point->y;
        centerPoint.z += point->z;   
    }
    centerPoint.x = centerPoint.x / points.size();
    centerPoint.y = centerPoint.y / points.size();
    centerPoint.z = centerPoint.z / points.size();

    //2) 协方差矩阵cov
	for(std::vector<pcl::PointXYZI>::iterator point = points.begin(); point != points.end(); point ++)
	{
		float buf[3];
		buf[0] = point->x - centerPoint.x;
		buf[1] = point->y - centerPoint.y;
		buf[2] = point->z - centerPoint.z;
		for(int pp=0; pp<3; pp++)
		{
			for(int l=0; l<3; l++)
			{
				cov[pp*3+l] += buf[pp]*buf[l];
			}
		}
	}
    
    return cloud;
}