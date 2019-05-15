#include ".\point3d.h"
#include <math.h>

//////////////////////////////////////////////////////////////////////////
/////////// 默认构造函数 ///////////
CPoint3d::CPoint3d(void)
:x(0),y(0),z(0)
{
}

CPoint3d::CPoint3d(float x,float y,float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

// 析构函数
CPoint3d::~CPoint3d(void)
{
}

// 复制构造函数
CPoint3d::CPoint3d(const CPoint3d& tmpP)
{
	this->x = tmpP.x;
	this->y = tmpP.y;
	this->z = tmpP.z;
}
//////////////////////////////////////////////////////////////////////////
/////////// 重载运算符为成员函数 ///////////
CPoint3d & CPoint3d::operator += ( const CPoint3d & p)
{
	this->x += p.x;
	this->y += p.y;
	this->z += p.z;
	return *this;
}
CPoint3d & CPoint3d::operator -= (const CPoint3d & p)
{
	this->x -= p.x;
	this->y -= p.y;
	this->z -= p.z;
	return *this;
}
CPoint3d & CPoint3d::operator *= ( float s )
{
	this->x *= s;
	this->y *= s;
	this->z *= s;
	return *this;
}
CPoint3d & CPoint3d::operator /= ( float s )
{
	this->x /= s;
	this->y /= s;
	this->z /= s;
	return *this;
}

// 重载运算符为友元函数
CPoint3d operator + (const CPoint3d &p1 ,const CPoint3d &p2)	// 相加
{	
	CPoint3d po;
	po.x = p1.x + p2.x;
	po.y = p1.y + p2.y;
	po.z = p1.z + p2.z;
	return po;
}
CPoint3d operator - (const CPoint3d &p1 ,const CPoint3d &p2)	// 
{	
	CPoint3d po;
	po.x = p1.x - p2.x;
	po.y = p1.y - p2.y;
	po.z = p1.z - p2.z;
	return po;
}
CPoint3d operator * ( const CPoint3d &p , float s )
{
	CPoint3d po;
	po.x = p.x * s;
	po.y = p.y * s;
	po.z = p.z * s;
	return po;
}

CPoint3d operator / (const CPoint3d &p ,float num)			// 除以一个整数
{		
	if(num != 0)
	{   
		CPoint3d po;
		po.x = p.x / num;
		po.y = p.y / num;
		po.z = p.z / num;
		return po;
	}
	else 
	{
		//AfxMessageBox(_T("错误：0不能做除数！"));
		return CPoint3d(0,0,0);
	}
}
bool operator <  ( const CPoint3d &p1 ,const CPoint3d &p2 )
{
	return	(p1.z != p2.z)?(p1.z < p2.z):
			(p1.y != p2.y)?(p1.y < p2.y):
					       (p1.x < p2.x);
}
bool operator >  ( const CPoint3d & p1, const CPoint3d &p2)
{
	return	(p1.z != p2.z)?(p1.z > p2.z):
			(p1.y != p2.y)?(p1.y > p2.y):
					       (p1.x > p2.x);
}
bool operator == (const CPoint3d &p1,const CPoint3d &p2)	// 相等
{
	if(p1.x==p2.x && p1.y==p2.y && p1.z==p2.z)
		return true;
	else 
		return false;
}
bool operator != (const CPoint3d &p1,const CPoint3d &p2)	// 不等
{
	if(p1.x==p2.x && p1.y==p2.y && p1.z==p2.z)
		return false;
	else 
		return true;
}
float operator *(const CPoint3d &p1, const CPoint3d &p2)
{
	return (p1.x*p2.x + p1.y*p2.y + p1.z*p2.z);
}
//////////////////////////////////////////////////////////////////////////
// 点到原点的距离
float CPoint3d::Dist(void)
{
	return sqrt(x*x+y*y+z*z);
}

// 点到原点的距离
float CPoint3d::Dist(const CPoint3d &p1)
{
	return sqrt((x-p1.x)*(x-p1.x)+(y-p1.y)*(y-p1.y)+(z-p1.z)*(z-p1.z));
}

CPoint3d & CPoint3d::Normalize()
{
	float n = float(sqrt(x*x+y*y+z*z));
	if(n > float(0))
	{ x /= n; y /= n; z /= n;}
	return *this;
}

float CPoint3d::dot(const CPoint3d &p)const
{
	return (*this) * p;
}
float  CPoint3d::DistSquare()
{
	return (x*x+y*y+z*z);
}
