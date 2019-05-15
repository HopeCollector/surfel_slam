#include "ComputeBox.h"
#include <cmath>

//������Ƶı߿򣨶�ά���Σ������Ǹ߶ȣ�,������
// ���룺the_points���洢��ѵ��Ƶ�һά���飻
// �����_lenth�����εĳ��� _width�����εĿ�
//       _len_dir�����γ�������ֱ�ߵķ��������� 
//       _wid_dir�����ζ̱ߣ�������ֱ�ߵķ�����������Ȼ����_len_dir�Ǵ�ֱ�ģ���
void ComputeBox::ComputeTheBox(vector<CPoint3d> &the_points, 
			                   float &_lenth, float &_width, CPoint3d &_len_dir, CPoint3d &_wid_dir)
{   
	//1)���ĵ�center
	center = CPoint3d(0,0,0);
	int cnt = 0;
	for (int i=0; i<the_points.size(); i++)
	{
		if(the_points[i].x==0 && the_points[i].y==0)  //������Ч��
			continue;
		center += the_points[i];
		cnt++;
	}
	center /= (float)cnt;

	//2) Э�������cov
	double cov[9] = {0.0}; //���Э�������
	for(int i=0; i<the_points.size(); i++)
	{
		if(the_points[i].x==0 && the_points[i].y==0)  //������Ч��
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

	//3)����cov������ֵ����������
	//---- �����������ֵΪcov[0]��cov[4]��cov[8]����Ӧ��
	//     ����������v��
	double v[9] = {0.0}; //�����������
	eejcb(cov, 3, v, 0.001, 1000);

	//4)������������������ֵ��С�������У���������������Ӧ��
	double eigenvalues[3] = {cov[0], cov[4], cov[8]};
	CPoint3d eigenvectors[3] = {CPoint3d(v[0], v[3], v[6]), CPoint3d(v[1], v[4], v[7]), CPoint3d(v[2], v[5], v[8])};
	for (int i=2; i>0; i--)  //ð������
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

	//5)���ս��
	//ע���ɶ������ݾ���õ�������ʵ����Լ��sqrt(��Ӧ����ֵ/�����)��3.52��
	
	_lenth = sqrt(eigenvalues[2]/cnt) * 3.52;
	_width = sqrt(eigenvalues[1]/cnt) * 3.52;
	_len_dir = eigenvectors[2];
	_wid_dir = eigenvectors[1];

}


//��ʵ�Գƾ��������ֵ�������������Ÿ�ȷ� 
//�����Ÿ��(Jacobi)������ʵ�Գƾ����ȫ������ֵ���������� 
//����ֵС��0��ʾ��������jt����δ�ﵽ����Ҫ�� 
//����ֵ����0��ʾ�������� 
//a-����Ϊn*n�����飬���ʵ�Գƾ��󣬷���ʱ�Խ��ߴ��n������ֵ 
//n-����Ľ��� 
//u-����Ϊn*n�����飬������������(���д洢) 
//eps-���ƾ���Ҫ�� 
//jt-���ͱ������������������� 
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