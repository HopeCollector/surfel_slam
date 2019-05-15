#include "ComputeBox.h"
#include <cmath>

//计算点云的边框（二维矩形，不考虑高度）,参数：
// 输入：the_points—存储这堆点云的一维数组；
// 输出：_lenth—矩形的长， _width—矩形的宽；
//       _len_dir—矩形长边所在直线的方向向量， 
//       _wid_dir—矩形短边（宽）所在直线的方向向量（当然，和_len_dir是垂直的）；
void ComputeBox::ComputeTheBox(vector<CPoint3d> &the_points, 
			                   float &_lenth, float &_width, CPoint3d &_len_dir, CPoint3d &_wid_dir)
{   
	//1)中心点center
	center = CPoint3d(0,0,0);
	int cnt = 0;
	for (int i=0; i<the_points.size(); i++)
	{
		if(the_points[i].x==0 && the_points[i].y==0)  //过滤无效点
			continue;
		center += the_points[i];
		cnt++;
	}
	center /= (float)cnt;

	//2) 协方差矩阵cov
	double cov[9] = {0.0}; //存放协方差矩阵
	for(int i=0; i<the_points.size(); i++)
	{
		if(the_points[i].x==0 && the_points[i].y==0)  //过滤无效点
			continue;

		float buf[3];
		buf[0] = the_points[i].x - center.x;
		buf[1] = the_points[i].y - center.y;
		buf[2] = 0;
		for(int pp=0; pp<3; pp++)
		{
			for(int l=0; l<3; l++)
			{
				cov[pp*3+l] += buf[pp]*buf[l];
			}
		}
	}

	//3)计算cov的特征值、特征向量
	//---- 计算出的特征值为cov[0]、cov[4]、cov[8]；对应的
	//     特征向量在v中
	double v[9] = {0.0}; //存放特征向量
	eejcb(cov, 3, v, 0.001, 1000);

	//4)整理结果，将三个特征值从小到大排列，并与特征向量对应好
	double eigenvalues[3] = {cov[0], cov[4], cov[8]};
	CPoint3d eigenvectors[3] = {CPoint3d(v[0], v[3], v[6]), CPoint3d(v[1], v[4], v[7]), CPoint3d(v[2], v[5], v[8])};
	for (int i=2; i>0; i--)  //冒泡排序
	{
		for (int j=0; j<i; j++)
		{
			if (eigenvalues[j] > eigenvalues[j+1])
			{
				double tmpVa = eigenvalues[j];
				CPoint3d tmpVe = eigenvectors[j];
				eigenvalues[j] = eigenvalues[j+1];
				eigenvectors[j] = eigenvectors[j+1];
				eigenvalues[j+1] = tmpVa;
				eigenvectors[j+1] = tmpVe;
			}
		}
	}

	//5)最终结果
	//注：由多组数据经验得到——真实长度约是sqrt(对应特征值/点个数)的3.52倍
	
	_lenth = sqrt(eigenvalues[2]/cnt) * 3.52;
	_width = sqrt(eigenvalues[1]/cnt) * 3.52;
	_len_dir = eigenvectors[2];
	_wid_dir = eigenvectors[1];

}


//求实对称矩阵的特征值及特征向量的雅格比法 
//利用雅格比(Jacobi)方法求实对称矩阵的全部特征值及特征向量 
//返回值小于0表示超过迭代jt次仍未达到精度要求 
//返回值大于0表示正常返回 
//a-长度为n*n的数组，存放实对称矩阵，返回时对角线存放n个特征值 
//n-矩阵的阶数 
//u-长度为n*n的数组，返回特征向量(按列存储) 
//eps-控制精度要求 
//jt-整型变量，控制最大迭代次数 
int ComputeBox::eejcb(double a[], int n, double v[], double eps, int jt) 
{ 
	int i,j,p,q,u,w,t,s,l; 
	double fm,cn,sn,omega,x,y,d; 
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