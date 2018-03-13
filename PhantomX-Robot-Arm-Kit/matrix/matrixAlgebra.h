//////////////////////////////////////////////////////
// Vector and Matrix files for Robotics applications
// 
// Copyright (c) 2002-2010. All Rights Reserved.
// Division of Applied Robot Technology, KITECH
// Web: http://www.orobot.net
// Written by KwangWoong Yang<page365@gmail.com>
//
// Collection of Mathematical functions

#pragma	once

#include "mat/matrix.h"

class dMatrix : public matrix<double> 
{
public:
	dMatrix (const dMatrix &m): matrix<double> (m) { }
	dMatrix (const matrix<double> &m): matrix<double> (m) { }
	dMatrix (size_t row = 1, size_t col = 1): matrix<double> (row, col) { }

	size_t rowno () const { return RowNo(); }
	size_t colno () const { return ColNo(); }

	size_t size()   const { return RowNo()*ColNo(); }
	void resize(size_t row, size_t col) { SetSize (row, col); }

	void unit () { Unit (); }
	void zero () { Null (); }

	dMatrix operator = (const dMatrix &m)
	{
		if (this != &m) {
			matrix<double>::operator = (m);
		}
		return *this;
	}
};

class dVector : public matrix<double> 
{
public:
	dVector (const dVector &m): matrix<double> (m) { }
	dVector (const matrix<double> &m): matrix<double> (m) { }
	dVector (size_t size = 3): matrix<double> (size, 1) { }

	size_t size()   const { return RowNo()*ColNo(); }
	void resize(size_t size) { SetSize (size, 1); }

	void zero () { Null (); }

	double operator[] (size_t i)  const 
    {
		return (*this)(i,0);
	}

	double& operator[] (size_t i)
    {
		return (*this)(i,0);
	}

	dVector operator = (const dVector &m)
	{
		if (this != &m) {
			matrix<double>::operator = (m);
		}
		return *this;
	}
};


// dVector의 내적(dot product)을 계산한다.
inline double Dot (const dVector &v1, const dVector &v2)
{
	double	v = 0.;
	for(unsigned int i=0; i<v1.size(); ++i) {
		v += v1[i] * v2[i];
	}

	return v;
}

// dVector의 외적(cross product)을 계산한다.
inline dVector Cross (const dVector &v1, const dVector &v2)
{
	dVector v(3);
	v[0] = v1[1]*v2[2] - v1[2]*v2[1];
	v[1] = v1[2]*v2[0] - v1[0]*v2[2];
	v[2] = v1[0]*v2[1] - v1[1]*v2[0];

	return v;
}

// Vector의 크기를 2-norm으로 계산한다
inline double Norm2 (const dVector &v)
{
	double s = 0.;
	for (int i=0; i<(int)v.size(); ++i) {
		s += v[i]*v[i];
	}
	return sqrt(s);
}

inline void Set (dVector &V, double v0, double v1, double v2)
{
	if (V.size() != 3) {
		V.resize (3);
	}

	V[0] = v0;
	V[1] = v1;
	V[2] = v2;
}

inline dMatrix Matrix3x1(double e0, double e1, double e2)
{
	dMatrix m(3,1);
	m(0,0) = e0;
	m(1,0) = e1;
	m(2,0) = e2;

	return m;
}

inline dMatrix RotationMatrix (double phi, double theta, double psi)
{
	double sphi = sin(phi),		cphi = cos(phi);
	double stht = sin(theta),	ctht = cos(theta);
	double spsi = sin(psi),		cpsi = cos(psi);

	dMatrix A(3,3);

	/*
	// 변환 메트릭스: Rx(phi) x Ry(theta) x Rz(psi)
	A(0, 0) =  ctht*cpsi;				A(0, 1) = -ctht*spsi;				A(0, 2) =  stht;		
	A(1, 0) =  sphi*stht*cpsi + cphi*spsi;	A(1, 1) = -sphi*stht*spsi + cphi*cpsi;	A(1, 2) = -sphi*ctht;	
	A(2, 0) = -cphi*stht*cpsi + sphi*spsi;	A(2, 1) =  cphi*stht*spsi + sphi*cpsi;	A(2, 2) =  cphi*ctht;	
	*/

	// 변환 메트릭스: Rz(psi) x Ry(theta) x Rx(phi)
	A(0,0) = cpsi*ctht;	A(0,1) = cpsi*stht*sphi - spsi*cphi;	A(0,2) = cpsi*stht*cphi + spsi*sphi;	
	A(1,0) = spsi*ctht;	A(1,1) = spsi*stht*sphi + cpsi*cphi;	A(1,2) = spsi*stht*cphi - cpsi*sphi;	
	A(2,0) = -stht;		A(2,1) = ctht*sphi;						A(2,2) = ctht*cphi;						

	return A;
}

