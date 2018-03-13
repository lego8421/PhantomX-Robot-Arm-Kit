//////////////////////////////////////////////////////
// OpenGL files for Robotics applications
// 
// Copyright (c) 2002-2010. All Rights Reserved.
// Division of Applied Robot Technology, KITECH
// Web: http://www.orobot.net
// Written by KwangWoong Yang<page365@gmail.com>
//

#pragma once

#include "oglDef.h"
#include <vector>

using namespace std;

extern void oglPlane(double size, double stride);
extern void oglCoordinate(double size);

extern void oglLine (double sx, double sy, double sz, double ex, double ey, double ez);
extern void oglLineStrip (vector <CPoint3d> &line);
extern void oglSphere (double radius); 
extern void oglSphere (double rx, double ry, double rz);
extern void oglCapsule (double height, double radius);
extern void oglCylinder (double height, double radius);
extern void oglCylinder (double height, double baseRadius, double topRadius, double xyRatio);
extern void oglBox (double dx, double dy, double dz);

