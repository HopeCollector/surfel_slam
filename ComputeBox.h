#pragma once

#include "Point3d.h"
#include <vector>
using namespace std;

class ComputeBox
{
public:
	//������Ƶı߿򣨶�ά���Σ������Ǹ߶ȣ�,������
	// ���룺the_points���洢��ѵ��Ƶ�һά���飻
	// �����_lenth�����εĳ��� _width�����εĿ�
	//       _len_dir�����γ�������ֱ�ߵķ��� 
	//       _wid_dir�����ζ̱ߣ�������ֱ�ߵķ��򣨵�Ȼ����_len_dir�Ǵ�ֱ�ģ���
 	void ComputeTheBox(vector<CPoint3d> &the_points, 
					   float &_lenth, float &_width, CPoint3d &_len_dir, CPoint3d &_wid_dir);

private:
	//������������ֵ����������
	int eejcb(double a[], int n, double v[], double eps, int jt);

	CPoint3d center; //���ĵ�
};