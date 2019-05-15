#pragma once

#include "Point3d.h"
#include <vector>
using namespace std;

class ComputeBox
{
public:
	//计算点云的边框（二维矩形，不考虑高度）,参数：
	// 输入：the_points—存储这堆点云的一维数组；
	// 输出：_lenth—矩形的长， _width—矩形的宽；
	//       _len_dir—矩形长边所在直线的方向， 
	//       _wid_dir—矩形短边（宽）所在直线的方向（当然，和_len_dir是垂直的）；
 	void ComputeTheBox(vector<CPoint3d> &the_points, 
					   float &_lenth, float &_width, CPoint3d &_len_dir, CPoint3d &_wid_dir);

private:
	//计算矩阵的特征值、特征向量
	int eejcb(double a[], int n, double v[], double eps, int jt);

	CPoint3d center; //中心点
};