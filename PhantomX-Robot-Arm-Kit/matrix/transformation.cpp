//////////////////////////////////////////////////////
// Vector and Matrix files for Robotics applications
// 
// Copyright (c) 2002-2010. All Rights Reserved.
// Division of Applied Robot Technology, KITECH
// Web: http://www.orobot.net
// Written by KwangWoong Yang<page365@gmail.com>
//

#include "transformation.h"

CTransformMatrix::CTransformMatrix () 
: dMatrix (4, 4)
{
	unit ();
}

CTransformMatrix::CTransformMatrix (double x, double y, double z, double phi, double theta, double psi)
: dMatrix (4, 4)
{
	dMatrix &A = *this;

	double sphi = sin(phi),		cphi = cos(phi);
	double stht = sin(theta),	ctht = cos(theta);
	double spsi = sin(psi),		cpsi = cos(psi);

	// transformation matrix: Trans(x,y,z) x Rz(psi) x Ry(theta) x Rx(phi)
	// 좌표 변환 순서:
	// 1. x, y, z 만큼 위치를 이동한다.
	// 2. z축으로 psi 만큼 회전한다.
	// 3. y축으로 theta 만큼 회전한다.
	// 4. x축으로 phi 만큼 회전한다.
	A(0,0) = cpsi*ctht;	A(0,1) = cpsi*stht*sphi - spsi*cphi;	A(0,2) = cpsi*stht*cphi + spsi*sphi;	A(0,3) = x;
	A(1,0) = spsi*ctht;	A(1,1) = spsi*stht*sphi + cpsi*cphi;	A(1,2) = spsi*stht*cphi - cpsi*sphi;	A(1,3) = y;
	A(2,0) = -stht;		A(2,1) = ctht*sphi;						A(2,2) = ctht*cphi;						A(2,3) = z;
	A(3,0) =  0;		A(3,1) =  0;							A(3,2) =  0;							A(3,3) = 1; 

	/*
	// transformation matrix: Trans(x,y,z) x Rx(phi) x Ry(theta) x Rz(psi)
	// 좌표 변환 순서:
	// 1. x, y, z 만큼 위치를 이동한다.
	// 2. x축으로 phi 만큼 회전한다.
	// 3. y축으로 theta 만큼 회전한다.
	// 4. z축으로 psi 만큼 회전한다.
	A(0,0) =  ctht*cpsi;					A(0,1) = -ctht*spsi;					A(0,2) =  stht;		A(0,3) = x;
	A(1,0) =  sphi*stht*cpsi + cphi*spsi;	A(1,1) = -sphi*stht*spsi + cphi*cpsi;	A(1,2) = -sphi*ctht;	A(1,3) = y;
	A(2,0) = -cphi*stht*cpsi + sphi*spsi;	A(2,1) =  cphi*stht*spsi + sphi*cpsi;	A(2,2) =  cphi*ctht;	A(2,3) = z;
	A(3,0) =  0;							A(3,1) =  0;							A(3,2) =  0;			A(3,3) = 1; 
	*/
}

dVector CTransformMatrix::GetRotation (int axis) const
{
	const dMatrix &A = *this;
	dVector v(3);

	v[0] = A(0,axis);
	v[1] = A(1,axis);
	v[2] = A(2,axis);

	return v;
}

dVector CTransformMatrix::GetPosition () const
{
	const dMatrix &A = *this;
	dVector v(3);

	v[0] = A(0,3);
	v[1] = A(1,3);
	v[2] = A(2,3);

	return v;
}

dVector CTransformMatrix::GetOrientation () const
{
	const dMatrix &A = *this;
	dVector v(3);

	/*
	v[0] = atan2(-A(1,2), A(2,2)); 
	v[1] = atan2( A(0,2), sqrt(A(1,2)*A(1,2) + A(2,2)*A(2,2)) ); 
	v[2] = atan2(-A(0,1), A(0,0)); 
	*/
	v[0] = atan2( A(2,1), A(2,2)); 
	v[1] = atan2(-A(2,0), sqrt(A(2,1)*A(2,1) + A(2,2)*A(2,2)) ); 
	v[2] = atan2( A(1,0), A(0,0)); 

	return v;
}

dVector CTransformMatrix::GetOrientation2 () const
{
	const dMatrix &A = *this;
	dVector v(3);

	// theta in the range (pi/2, 3pi/2)
	/*
	v[0] = atan2( A(1,2), -A(2,2)); 
	v[1] = atan2( A(0,2), -sqrt(A(1,2)*A(1,2) + A(2,2)*A(2,2)) ); 
	v[2] = atan2( A(0,1), -A(0,0)); 
	*/
	v[0] = atan2(-A(2,1),-A(2,2)); 
	v[1] = atan2(-A(2,0),-sqrt(A(2,1)*A(2,1) + A(2,2)*A(2,2)) ); 
	v[2] = atan2(-A(1,0),-A(0,0)); 

	return v;
}

dVector CTransformMatrix::GetPositionOrientation () const
{
	const dMatrix &A = *this;
	dVector v(6);

	v[0] = A(0,3);
	v[1] = A(1,3);
	v[2] = A(2,3);
	// theta in the range (-pi/2, pi/2)
	/*
	v[3] = atan2(-A(1,2), A(2,2)); 
	v[4] = atan2( A(0,2), sqrt(A(1,2)*A(1,2) + A(2,2)*A(2,2)) ); 
	v[5] = atan2(-A(0,1), A(0,0)); 
	*/
	v[3] = atan2( A(2,1), A(2,2)); 
	v[4] = atan2(-A(2,0), sqrt(A(2,1)*A(2,1) + A(2,2)*A(2,2)) ); 
	v[5] = atan2( A(1,0), A(0,0)); 

	return v;
}

dVector CTransformMatrix::GetPositionOrientation2 () const 
{
	const dMatrix &A = *this;
	dVector v(6);

	v[0] = A(0,3);
	v[1] = A(1,3);
	v[2] = A(2,3);
	// theta in the range (-pi/2, pi/2)
	/*
	v[3] = atan2( A(1,2), -A(2,2)); 
	v[4] = atan2( A(0,2), -sqrt(A(1,2)*A(1,2) + A(2,2)*A(2,2)) ); 
	v[5] = atan2( A(0,1), -A(0,0)); 
	*/
	v[3] = atan2(-A(2,1),-A(2,2)); 
	v[4] = atan2(-A(2,0),-sqrt(A(2,1)*A(2,1) + A(2,2)*A(2,2)) ); 
	v[5] = atan2(-A(1,0),-A(0,0)); 

	return v;
}

