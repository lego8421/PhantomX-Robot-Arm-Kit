//////////////////////////////////////////////////////
// Kinematics files for Robotics applications
// 
// Copyright (c) 2002-2010. All Rights Reserved.
// Division of Applied Robot Technology, KITECH
// Web: http://www.orobot.net
// Written by KwangWoong Yang<page365@gmail.com>
//

#include "posOriInverse.h"
#include "matrix/transformation.h"
#include "matrix/mathDef.h"
#include "quaternion.h"

CPosOriInverse::CPosOriInverse (int inverseAxis)
: CKinematics(6), _inverseAxis(inverseAxis)
{ 
}

CPosOriInverse::~CPosOriInverse ()
{
}

dMatrix CPosOriInverse::Forward ()
{
	CTransformMatrix T;

	for (unsigned int i=0; i<_jointList.size (); ++i) {
		JointInfo *joint = _jointList[i];

		dMatrix A = joint->TransformationMatrix ();

        joint->a = T.GetRotation (joint->axis);
        joint->p = T.GetPosition ();

		T *= A;
	}
	_current = T.GetPositionOrientation ();

	return T;
}

dMatrix CPosOriInverse::GetEulerVel2AngularVel (double phi, double theta, double psi)
{
	double cpsi = cos(psi);
	double spsi = sin(psi);
	double cthe = cos(theta);
	double sthe = sin(theta);

	dMatrix C(3,3);
	C(0,0) = cpsi*cthe;	C(0,1) = -spsi;	C(0,2) = 0;
	C(1,0) = spsi*cthe;	C(1,1) = cpsi;	C(1,2) = 0;
	C(2,0) = -sthe;		C(2,1) = 0;		C(2,2) = 1;

	return C;
}

dMatrix CPosOriInverse::Jacobian()
{
	//Position Jacobian 
	dMatrix J(6, _jointList.size());
	J.zero ();

	dVector currentPos (3);
	currentPos[0] = _current[0];
	currentPos[1] = _current[1]; 
	currentPos[2] = _current[2]; 

	dMatrix Cinv = !GetEulerVel2AngularVel (_current[3], _current[4], _current[5]);

	for (unsigned int i=0; i<_jointList.size (); ++i) {
		JointInfo *joint = _jointList[i];

		if (joint->type == REVOLUTE_JOINT) {
			if (_inverseAxis & POSITION_ONLY) {
				//Position Jacobian 
				dVector pos = Cross (joint->a, currentPos - joint->p);
				J(0,i) = pos[0];
				J(1,i) = pos[1];
				J(2,i) = pos[2];
			}
			if (_inverseAxis & ORIENTATION_ONLY) {
				dMatrix o = Cinv*Matrix3x1(joint->a[0], joint->a[1], joint->a[2]);
				//Orientation Jacobian 
				J(3,i) = o(0,0);
				J(4,i) = o(1,0);
				J(5,i) = o(2,0);
			}
		}
		else if (joint->type == PRISMATIC_JOINT) {
			if (_inverseAxis & POSITION_ONLY) {
				//Position Jacobian 
				J(0,i) = joint->a[0];
				J(1,i) = joint->a[1];
				J(2,i) = joint->a[2];
			}
			if (_inverseAxis & ORIENTATION_ONLY) {
				//Orientation Jacobian 
				J(3,i) = 0.;
				J(4,i) = 0.;
				J(5,i) = 0.; 
			}
		}
	}
	return J;
}

dVector CPosOriInverse::Error ()
{
	dVector error(6);
	error.zero ();
	
	if (_inverseAxis & POSITION_ONLY) {
		error[0] = _desired[0] - _current[0];
		error[1] = _desired[1] - _current[1];
		error[2] = _desired[2] - _current[2];
	}

	if (_inverseAxis & ORIENTATION_ONLY) {
		error[3] = DeltaRad (_desired[3], _current[3]);
		error[4] = DeltaRad (_desired[4], _current[4]);
		error[5] = DeltaRad (_desired[5], _current[5]);
	}

	return error;
}

void CPosOriInverse::SetDesired (double x, double y, double z, double phi, double theta, double psi)
{
	_desired[0] = x;		
	_desired[1] = y;		
	_desired[2] = z; 
	_desired[3] = DeltaRad (phi, 0);
	_desired[4] = DeltaRad (theta, 0);
	_desired[5] = DeltaRad (psi, 0);
}
