//////////////////////////////////////////////////////
// OpenGL files for Robotics applications
// 
// Copyright (c) 2002-2010. All Rights Reserved.
// Division of Applied Robot Technology, KITECH
// Web: http://www.orobot.net
// Written by KwangWoong Yang<page365@gmail.com>
//

#pragma once

class CPoint3d {

public:
    CPoint3d() {}
    CPoint3d(double _x, double _y, double _z) : x(_x), y(_y), z(_z) { }
	double x, y, z;
};

