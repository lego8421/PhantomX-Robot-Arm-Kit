//////////////////////////////////////////////////////
// Kinematics files for Robotics applications
// 
// Copyright (c) 2002-2010. All Rights Reserved.
// Division of Applied Robot Technology, KITECH
// Web: http://www.orobot.net
// Written by KwangWoong Yang<page365@gmail.com>
//

#pragma	once

#include "kinematics.h"

#define POSITION_ONLY			1
#define ORIENTATION_ONLY		2
#define POSITION_ORIENTATION	(POSITION_ONLY|ORIENTATION_ONLY)

class CPosOriInverse : public CKinematics
{
public:
	CPosOriInverse (int inverseAxis = POSITION_ORIENTATION); 
	~CPosOriInverse (); 
	
    void SetDesired (double x, double y, double z, double phi, double theta, double psi);

	virtual dMatrix Forward ();
	virtual dVector Error ();
	virtual dMatrix Jacobian ();

private:
	dMatrix GetEulerVel2AngularVel (double phi, double theta, double psi);

	int _inverseAxis;
};
