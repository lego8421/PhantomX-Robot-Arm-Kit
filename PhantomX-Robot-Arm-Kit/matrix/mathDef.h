//////////////////////////////////////////////////////
// Math files for Robotics applications
// 
// Copyright (c) 2002-2010. All Rights Reserved.
// Division of Applied Robot Technology, KITECH
// Web: http://www.orobot.net
// Written by KwangWoong Yang<page365@gmail.com>
//
// Collection of Mathematical functions

#pragma once
#include <math.h>

#ifndef M_PI
#define M_PI		3.14159265358979323846	// pi 
#endif

#define _RAD2DEG	(180./M_PI)
#define _DEG2RAD	(M_PI/180.)


// a의 부호를 리턴한다.
inline double Sign (double a) 
{
	return (0. < a) ? 1. : ((a < 0) ? -1. : 0.);
}

// ang1과 ang2의 사잇각을 계산하여 리턴한다.
// 리턴 값은 -pi ~ pi 사이의 값이다.
inline double DeltaRad (double ang1, double ang2)
{
	double da = ang1 - ang2;
	if (-M_PI < da && da < M_PI) return da;
	else {
		da = fmod (da, 2*M_PI);
		if (M_PI <= da) return da - 2*M_PI;
		else if (da <= -M_PI) return da + 2*M_PI;
		else return da;
	}
	return da;
}

inline double Limit (double x, double lo, double hi)
{
	if (x < lo) return lo;
	if (x > hi) return hi;
	return x;
}