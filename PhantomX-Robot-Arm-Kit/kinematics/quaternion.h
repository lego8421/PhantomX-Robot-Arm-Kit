//////////////////////////////////////////////////////
// Quaternion files for Robotics applications
// 
// Copyright (c) 2002-2010. All Rights Reserved.
// Division of Applied Robot Technology, KITECH
// Web: http://www.orobot.net
// Written by KwangWoong Yang<page365@gmail.com>
//

#pragma once

#include "matrix/matrixAlgebra.h"

class Quaternion
{
public:
	Quaternion();
	Quaternion(const Quaternion &q);
	Quaternion(const double angle_in_rad, const dVector & axis);
	Quaternion(const double s, const double v1, const double v2, const double v3);
	Quaternion(double phi, double theta, double psi);

	Quaternion  operator+(const Quaternion & q)const;
	Quaternion  operator-(const Quaternion & q)const;
	Quaternion  operator*(const Quaternion & q)const;
	Quaternion  operator/(const Quaternion & q)const;

	Quaternion  conjugate()const;
	double      norm()const;
	Quaternion &unit(); 
	Quaternion  i()const;
	double      dot(const Quaternion & q)const;

	double      s() const { return _s; }			// Return scalar part.
	void        set_s (const double s){ _s = s; }	// Set scalar part.
	dVector     v() const { return _v; }			// Return vector part.
	void        set_v (const dVector & v);			// Set vector part.

	Quaternion & operator = (const Quaternion &rhs) 
	{ 
		if (this != &rhs) {
			_s = rhs.s(); 
			_v = rhs.v();
		}
		return *this;
	}

private:
	double _s;			// Quaternion scalar part.
	dVector _v;			// Quaternion vector part.
};

Quaternion  operator * (const double c, const Quaternion & rhs);
Quaternion  operator * (const Quaternion & lhs, const double c);
Quaternion  operator / (const double c, const Quaternion & rhs);
Quaternion  operator / (const Quaternion & lhs, const double c);
