//////////////////////////////////////////////////////
// Quaternion files for Robotics applications
// 
// Copyright (c) 2002-2010. All Rights Reserved.
// Division of Applied Robot Technology, KITECH
// Web: http://www.orobot.net
// Written by KwangWoong Yang<page365@gmail.com>
//

#include "quaternion.h"
#include <math.h>

#define EPSILON 0.0000001

Quaternion::Quaternion()
: _s(1.), _v(3)
{
	_v[0] = 0.;
	_v[1] = 0.;
	_v[2] = 0.;
}

Quaternion::Quaternion(const Quaternion &q)
{
	_s = q.s();
	_v = q.v();
}

Quaternion::Quaternion(const double angle, const dVector & axis)
{
    if(axis.size() != 3) {
		return;
	}

	// make sure axis is a unit vector
	_v = sin(angle/2) * axis/Norm2(axis);
	_s = cos(angle/2);
}

Quaternion::Quaternion(const double s_in, const double v1, const double v2, const double v3)
: _s(s_in), _v(3)
{
	_v[0] = v1;
	_v[1] = v2;
	_v[2] = v3;
}

inline double sgn (double a) 
{
	return (0. <= a) ? 1. : -1;
}

Quaternion::Quaternion(double phi, double theta, double psi)
: _s(0.), _v(3)
{
	Quaternion Qx (cos(phi/2),   sin(phi/2), 0,            0         );
	Quaternion Qy (cos(theta/2), 0,          sin(theta/2), 0         );
	Quaternion Qz (cos(psi/2),   0,          0,            sin(psi/2));

	/*
	*this = Qx*Qy*Qz;
	*/

	*this = Qz*Qy*Qx;
}

Quaternion Quaternion::operator+(const Quaternion & rhs)const
{
	Quaternion q;
	q._s = _s + rhs._s;
	q._v = _v + rhs._v;

	return q;
}

Quaternion Quaternion::operator-(const Quaternion & rhs)const
{
	Quaternion q;
	q._s = _s - rhs._s;
	q._v = _v - rhs._v;

	return q;
}

Quaternion Quaternion::operator*(const Quaternion & rhs)const
{
	Quaternion q;
	q._s = _s * rhs._s - Dot (_v, rhs._v);
	q._v = _s * rhs._v + rhs._s * _v + Cross (_v, rhs._v);

	return q;
}


Quaternion Quaternion::operator/(const Quaternion & rhs)const
{
	return *this*rhs.i();
}


Quaternion Quaternion::conjugate()const
{
	Quaternion q;
	q._s = _s;
	q._v = -1.*_v;

	return q;
}

double Quaternion::norm()const 
{ 
	return( sqrt(_s*_s + Dot (_v, _v)) );
}

Quaternion & Quaternion::unit()
{
	double tmp = norm();
	if(tmp > EPSILON) {
		_s = _s/tmp;
		_v = _v/tmp;
	}
	return *this;
}

Quaternion Quaternion::i()const 
{ 
	return conjugate()/norm();
}

double Quaternion::dot(const Quaternion & q)const
{
	return (_s*q._s + _v[0]*q._v[0] + _v[1]*q._v[1] + _v[2]*q._v[2]);
}

void Quaternion::set_v(const dVector & v)
{
	if(v.size() == 3) {
		_v = v;
	}
	else {
        return;
	}
}

Quaternion operator*(const double c, const Quaternion & q)
{
	Quaternion out;
	out.set_s(q.s() * c);
	out.set_v(q.v() * c);
	return out;
}

Quaternion operator*(const Quaternion & q, const double c)
{
	return operator*(c, q);
}

Quaternion operator/(const double c, const Quaternion & q)
{
	Quaternion out;
	out.set_s(q.s() / c);
	out.set_v(q.v() / c);
	return out;
}

Quaternion operator/(const Quaternion & q, const double c)
{
	return operator/(c, q);
}
