//////////////////////////////////////////////////////////////////////////
// CPoint3d 3D点类，用来保存三维空间中点的信息,一般C++类
// CPoint3d类中支持3d点的 +, -, /， ==, <, >等操作符的重载
//
// 设计者：郑克强	2008.12.03 
//
// 修改：修改了重载操作符函数  何国建 2011.7.14
//////////////////////////////////////////////////////////////////////////

#pragma once

class CPoint3d
{
public:
	float x;	// 3d点的x坐标
	float y;	// 3d点的y坐标
	float z;	// 3d点的z坐标

public:
	// 构造函数
	CPoint3d(void);
	CPoint3d(float x,float y,float z);
	~CPoint3d(void);
	// 复制构造函数
	CPoint3d(const CPoint3d& tmpP);

	// 重载运算符为成员函数
	CPoint3d & operator += ( const CPoint3d & p);
    CPoint3d & operator -= ( const CPoint3d & p);
    CPoint3d & operator *= ( float s );
    CPoint3d & operator /= ( float s );
	// 重载运算符为友元函数
	friend bool operator >  (const CPoint3d &p1,const CPoint3d &p2);
	friend bool operator <  (const CPoint3d &p1,const CPoint3d &p2);
	friend bool operator == (const CPoint3d &p1,const CPoint3d &p2);
	friend bool operator != (const CPoint3d &p1,const CPoint3d &p2);	
	friend CPoint3d operator + (const CPoint3d &p1 ,const CPoint3d &p2);
	friend CPoint3d operator - (const CPoint3d &p1 ,const CPoint3d &p2);
	friend CPoint3d operator * ( const CPoint3d &p , float s );
	friend float operator *(const CPoint3d &p1, const CPoint3d &p2);
	friend CPoint3d operator / (const CPoint3d &p ,float num);

	float dot(const CPoint3d &p)const;
	// 点到原点的距离
	float Dist(void);
	// 点到其他点的距离
	float Dist(const CPoint3d &p1);
	CPoint3d & Normalize();

	//到原点的距离的平方,为了比较距离更快些
	float DistSquare();
};