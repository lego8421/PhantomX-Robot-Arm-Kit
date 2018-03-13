//////////////////////////////////////////////////////
// Kinematics files for Robotics applications
// 
// Copyright (c) 2002-2010. All Rights Reserved.
// Division of Applied Robot Technology, KITECH
// Web: http://www.orobot.net
// Written by KwangWoong Yang<page365@gmail.com>
//

#pragma once

#include <vector>

#include "matrix/transformation.h"
#include "matrix/mathDef.h"

using namespace std;


enum eJointType { 
	FIXED_JOINT,		// 고정된 조인트: 좌표를 이동하기위해 사용한다.
	REVOLUTE_JOINT,		// 회전 운동 조인트
	PRISMATIC_JOINT,	// 직선 운동 조인트
};

struct JointInfo {
	// 조인트의 초기 설정치들
	eJointType type;			// 종류
	int axis;					// 회전(직선) 운동 중심 축 지정 (0 - x축, 1 - y축, 2 - z축)

	double x, y, z;				// 이전 조인트 위치로부터 현재 조인트의 상대 위치
	double phi, theta, psi;		// 이전 조인트 자세로부터 현재 조인트의 상대 자세
	double radius;				// 링크의 굵기, OpenGL에서 그리기 위해서 사용함

	// 프로그램 실행 중 사용되는 변수들
	double q;					// 조인트의 현재 회전(직선) 위치
	double limit_lo, limit_hi;	// 조인트의 구동 범위 (limit_lo ~ limit_hi)
	double median;				// 조인트의 중간 값 (Null motion을 할당하기 위해 사용됨)
	double weight;				// 조인트의 가중치
	dVector a;					// 회전 축의 각속도
	dVector p;					// 전역 좌표계에서 회전 축의 위치

    JointInfo() : q(0.), limit_lo(-M_PI), limit_hi(M_PI), median(0.), weight(0.), a(3), p(3) { }

    CTransformMatrix TransformationMatrixQ();
    CTransformMatrix TransformationMatrix();
};


class CKinematics
{
public:
	CKinematics(int taskSize);
	~CKinematics(void);

    virtual dMatrix Forward() = 0;
    virtual dVector Error() = 0;
    virtual dMatrix Jacobian() = 0;

    dVector GetDesired() { return _desired; }	// 목적지의 위치를 반환한다.
    dVector GetCurrent() { return _current; }	// 현재 위치를 반환한다.

	// 조인트 목록 끝에 하나의 조인트를 더한다.
	void AttachJoint (eJointType type, int axis, double x, double y, double z, double phi, double theta, double psi, 
		double radius, double limit_lo = -M_PI, double limit_hi = M_PI, double median = 0., double weight = 1.);

	dVector SolveJTR (double minAlpha, double clampTask);					// Jacobian Transpose 방법으로 Inverse kinematics를 푼다.
	dVector SolveDLS (double dlsLambda, double clampTask, double clampNull);	// Damped least square 방법으로 Inverse kinematics를 푼다.

    dVector GetJointAngle();					// 현재 조인트 각도(q) 집합을 반환한다.
	void SetJointAngle(dVector &q);
    void Apply(dVector &dq);					// 각 조인트의 회전량(dq)를 현재 조인트 각도(q)에 더한다.

    vector<JointInfo *> &GetJointList() { return _jointList; }	// 조인트 정보를 담고있는 목록을 얻어온다. (그리기 위해 사용)

protected:
	vector<JointInfo *> _jointList;

	dVector _desired;
	dVector _current;

	void ClampMag(dVector &w, double maxMag);

	double WeightFunction (double lo, double hi, double q);
	dMatrix WeightMatrix (unsigned int dof);
	dVector NullSpaceVector ();
};
