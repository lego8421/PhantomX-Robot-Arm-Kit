//////////////////////////////////////////////////////
// Vector and Matrix files for Robotics applications
// 
// Copyright (c) 2002-2010. All Rights Reserved.
// Division of Applied Robot Technology, KITECH
// Web: http://www.orobot.net
// Written by KwangWoong Yang<page365@gmail.com>
//

#pragma once
#include "matrixAlgebra.h"

class CTransformMatrix : public dMatrix 
{
public:
	CTransformMatrix ();
	CTransformMatrix (const matrix<double> &m): dMatrix (m) { }
	CTransformMatrix (double x, double y, double z, double phi, double theta, double psi);

	dVector GetRotation (int axis) const;		// 변환 행렬에서 축에 대한 회전 성분 벡터를 반환한다.
	dVector GetPosition () const;				// 변환 행렬의 위치(x, y, z) 벡터를 반환한다.
	dVector GetOrientation () const;			// 변환 행렬의 방향(phi, theta, psi) 벡터를 반환한다. (theta: -pi/2 ~ pi/2)
	dVector GetOrientation2 () const;			// 변환 행렬의 방향(phi, theta, psi) 벡터를 반환한다. (theta:  pi/2 ~ 3*pi/2)
	dVector GetPositionOrientation () const;	// 변환 행렬의 위치와 방향 벡터를 반환한다. (theta: -pi/2 ~ pi/2)
	dVector GetPositionOrientation2 () const;	// 변환 행렬의 위치와 방향 벡터를 반환한다. (theta:  pi/2 ~ 3*pi/2)
};
