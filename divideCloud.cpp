#include "ioVis.h"
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <pcl/visualization/pcl_visualizer.h>
#include <math.h>

#define gridLength 1.0
#define Pos(x,y,z) (x + y*(xBlockNum) + z*(xBlockNum)*(yBlockNum))

struct ProTime{
	std::string proName;
	std::clock_t proTime;
};

typedef pcl::PointCloud<pcl::PointXYZI>::Ptr XYZICloudPtr;
typedef pcl::PointCloud<pcl::PointXYZI> XYZICloud;
typedef pcl::PointCloud<pcl::PointXYZINormal>::Ptr SurfelCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZINormal> SurfelCloud;
typedef std::vector<pcl::PointXYZI> Grid;

// 从 vector 中制作点云
XYZICloudPtr genCloud(Grid);
// 将栅格中的所有点转换为一个 Surfel 特征
pcl::PointXYZINormal genSurfel(Grid);
// 将实对称矩阵分解为特征向量和特征值
int eejcb(float a[], int n, float v[], float eps, int jt);

int main(int argc, char* argv[]){
    XYZICloudPtr cloud = getFile(argv[1]);
    SurfelCloudPtr surfelCloud(new SurfelCloud);
	std::vector<ProTime> totalRunTime(3);
	std::map<std::string,Grid> subClouds;
    float min_x = 0;
    float min_y = 0;
    float min_z = 0;

	totalRunTime[0].proName = "findBound";
	totalRunTime[1].proName = "gridMash";
	totalRunTime[2].proName = "genSurfel";

	// 找到点云数据的边界
	totalRunTime[0].proTime = clock();
    for (auto point : cloud->points) {
        min_x = point.x < min_x ? point.x : min_x;
        min_y = point.y < min_y ? point.y : min_y;
        min_z = point.z < min_z ? point.z : min_z;
    }
	totalRunTime[0].proTime = clock() - totalRunTime[0].proTime;
	cout << totalRunTime[0].proName << " time:" << totalRunTime[0].proTime << endl;

	// 将不同的点根据其坐标分配到不同的栅格里
	totalRunTime[1].proTime = clock();
    for(auto point : cloud->points){
        const int xPos = (point.x - min_x) / gridLength;
        const int yPos = (point.y - min_y) / gridLength;
        const int zPos = (point.z - min_z) / gridLength;
        subClouds["("+std::to_string(xPos)+","
					 +std::to_string(yPos)+","
					 +std::to_string(zPos)+")"].push_back(point);
    }
	totalRunTime[1].proTime = clock() - totalRunTime[1].proTime;
	cout << totalRunTime[1].proName << " time:" << totalRunTime[1].proTime << endl;

	// 将栅格中的所有点用一个带有 Surfel 特征的中心点代表
	totalRunTime[2].proTime = clock();
    for(auto subCloud : subClouds){
		if(subCloud.second.empty()) continue;
		surfelCloud->push_back(genSurfel(subCloud.second));
    }
	totalRunTime[2].proTime = clock() - totalRunTime[2].proTime;
	cout << totalRunTime[2].proName << " time:" << totalRunTime[2].proTime << endl;

    cout << "\nSurfel extract complete!\n" << "Surfel map size: " << surfelCloud->size() << endl;

	// 将统计数据写入 csv 文件
	std::ofstream csvFile;
	csvFile.open("div.csv",ios::app);
	csvFile << gridLength << "," << totalRunTime[0].proTime << "," << totalRunTime[1].proTime << "," << totalRunTime[2].proTime << endl;
	cout << "total run time:" << gridLength << "," << totalRunTime[0].proTime << "," << totalRunTime[1].proTime << "," << totalRunTime[2].proTime << endl;
	csvFile.close();

	// 显示部分
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> handler ("intensity");
    pcl::ModelCoefficients coeffs;

    handler.setInputCloud(cloud);
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZI>(cloud, handler, "xyziCloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "xyziCloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();


	// 将 Surfel 以圆锥的形式画出来，所有的实参都是测试出来的
    for(int i = 0; i < surfelCloud->size(); i++){
        pcl::PointXYZINormal p = surfelCloud->points[i];
        if(p.intensity == 0) continue;
        coeffs.values.clear();
        coeffs.values.push_back (p.x);// 三个数字表示圆锥的位置
        coeffs.values.push_back (p.y);
        coeffs.values.push_back (p.z);
        coeffs.values.push_back (p.normal_x*0.15); // 三个数字表示圆锥的朝向，同时用 sqrt(x*x + y*y + z*z) 计算圆锥的高度
        coeffs.values.push_back (p.normal_y*0.15);
        coeffs.values.push_back (p.normal_z*0.15);
        coeffs.values.push_back (std::atan(p.intensity*500)*50); // 圆锥的角度 [0～90]，arctan 是为了归一化
        viewer->addCone (coeffs, "cone" + std::to_string(i));
    }

    while (!viewer->wasStopped()){
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds (100000));
    }

    return 0;
}

XYZICloudPtr genCloud(Grid points){
    XYZICloudPtr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for(Grid::iterator point = points.begin(); point != points.end(); point ++){
        cloud->push_back(*point);
    }
    
    return cloud;
}

pcl::PointXYZINormal genSurfel(Grid points){
    if(points.empty()){
        pcl::PointXYZINormal point;
        point.normal_x = 0;
        point.normal_y = 0;
        point.normal_z = 0;
        return point;
    }
    
    pcl::PointXYZINormal centerPoint;
    float cov[9] = {0.0};
    float v[9] = {0.0};


    for(Grid::iterator point = points.begin(); point != points.end(); point ++){
        centerPoint.x += point->x;
        centerPoint.y += point->y;
        centerPoint.z += point->z;   
    }
    centerPoint.x = centerPoint.x / points.size();
    centerPoint.y = centerPoint.y / points.size();
    centerPoint.z = centerPoint.z / points.size();

    //2) 协方差矩阵cov
	for(Grid::iterator point = points.begin(); point != points.end(); point ++)
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
    for(int i = 0; i < 9; i++){
        cov[i] = cov[i] / points.size();
    }

    eejcb(cov, 3, v, 0.001, 1000);
    
    float eigenValue[3] = {cov[0], cov[4], cov[8]};
    float* eigenVector[3];
    eigenVector[0] = new float[3]{v[0],v[3],v[6]};
    eigenVector[1] = new float[3]{v[1],v[4],v[7]};
    eigenVector[2] = new float[3]{v[2],v[5],v[8]};

    for (int i=2; i>0; i--)  //冒泡排序从小到大
	{
		for (int j=0; j<i; j++)
		{
			if (eigenValue[j] > eigenValue[j+1])
			{
				float tmpVa = eigenValue[j];
				float* tmpVe = eigenVector[j];
				eigenValue[j] = eigenValue[j+1];
				eigenVector[j] = eigenVector[j+1];
				eigenValue[j+1] = tmpVa;
				eigenVector[j+1] = tmpVe;
			}
		}
	}

    centerPoint.normal_x = eigenVector[0][0];
    centerPoint.normal_y = eigenVector[0][1];
    centerPoint.normal_z = eigenVector[0][2];
    centerPoint.intensity = eigenValue[2] * eigenValue[1];

    return centerPoint;
}

// 将矩阵分解为特征值和特征向量
int eejcb(float a[], int n, float v[], float eps, int jt) { 
	int i,j,p,q,u,w,t,s,l; 
	float fm,cn,sn,omega,x,y,d; 
	l=1; 
	for (i=0; i<=n-1; i++) 
	{ 
		v[i*n+i]=1.0; 
		for (j=0; j<=n-1; j++) 
		{ 
			if (i!=j) 
			{ 
				v[i*n+j]=0.0; 
			} 
		} 
	} 
	while (1==1) 
	{ 
		fm=0.0; 
		for (i=0; i<=n-1; i++) 
		{ 
			for (j=0; j<=n-1; j++) 
			{ 
				d=fabs(a[i*n+j]); 
				if ((i!=j)&&(d>fm)) 
				{ 
					fm=d; 
					p=i; 
					q=j; 
				} 
			} 
		} 
		if (fm<eps) 
		{ 
			return(1); 
		} 
		if (l>jt) 
		{ 
			return(-1); 
		} 
		l=l+1; 
		u=p*n+q; 
		w=p*n+p; 
		t=q*n+p; 
		s=q*n+q; 
		x=-a[u]; 
		y=(a[s]-a[w])/2.0; 
		omega=x/sqrt(x*x+y*y); 
		if (y<0.0) 
		{ 
			omega=-omega; 
		} 
		sn=1.0+sqrt(1.0-omega*omega); 
		sn=omega/sqrt(2.0*sn); 
		cn=sqrt(1.0-sn*sn); 
		fm=a[w]; 
		a[w]=fm*cn*cn+a[s]*sn*sn+a[u]*omega; 
		a[s]=fm*sn*sn+a[s]*cn*cn-a[u]*omega; 
		a[u]=0.0; 
		a[t]=0.0; 
		for (j=0; j<=n-1; j++) 
		{ 
			if ((j!=p)&&(j!=q)) 
			{ 
				u=p*n+j; 
				w=q*n+j; 
				fm=a[u]; 
				a[u]=fm*cn+a[w]*sn; 
				a[w]=-fm*sn+a[w]*cn; 
			} 
		} 
		for (i=0; i<=n-1; i++) 
		{ 
			if ((i!=p)&&(i!=q)) 
			{ 
				u=i*n+p; 
				w=i*n+q; 
				fm=a[u]; 
				a[u]=fm*cn+a[w]*sn; 
				a[w]=-fm*sn+a[w]*cn; 
			} 
		} 
		for (i=0; i<=n-1; i++) 
		{ 
			u=i*n+p; 
			w=i*n+q; 
			fm=v[u]; 
			v[u]=fm*cn+v[w]*sn; 
			v[w]=-fm*sn+v[w]*cn; 
		} 
	} 
	return(1); 
} 