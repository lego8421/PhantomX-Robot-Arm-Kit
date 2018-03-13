//////////////////////////////////////////////////////
// Kinematics files for Robotics applications
// 
// Copyright (c) 2002-2010. All Rights Reserved.
// Division of Applied Robot Technology, KITECH
// Web: http://www.orobot.net
// Written by KwangWoong Yang<page365@gmail.com>
//

#include "kinematics.h"

CTransformMatrix JointInfo::TransformationMatrixQ()
{
	if (type == PRISMATIC_JOINT){
        if (axis == 0)      return CTransformMatrix(q, 0, 0, 0, 0, 0);
        else if (axis == 1) return CTransformMatrix(0, q, 0, 0, 0, 0);
        else if (axis == 2) return CTransformMatrix(0, 0, q, 0, 0, 0);
	}
	else if (type == REVOLUTE_JOINT) {
        if (axis == 0)      return CTransformMatrix(0, 0, 0, q, 0, 0);
        else if (axis == 1) return CTransformMatrix(0, 0, 0, 0, q, 0);
        else if (axis == 2) return CTransformMatrix(0, 0, 0, 0, 0, q);
	}

    return CTransformMatrix(0, 0, 0, 0, 0, 0);
}

CTransformMatrix JointInfo::TransformationMatrix()
{
    return TransformationMatrixQ() * CTransformMatrix(x, y, z, phi, theta, psi);
}


CKinematics::CKinematics(int taskSize) :
	_desired(taskSize), _current(taskSize)
{
    _desired.zero();
    _current.zero();
}

CKinematics::~CKinematics(void)
{
    for (vector<JointInfo *>::iterator it = _jointList.begin(); it != _jointList.end(); ++it) {
		delete *it;
	}
}

void CKinematics::AttachJoint (eJointType type, int axis, double x, double y, double z, double phi, double theta, double psi, 
							   double radius, double limit_lo, double limit_hi, double median, double weight)
{
	JointInfo *joint = new JointInfo;
	
	joint->type = type;
	joint->axis = axis;

	joint->x = x;
	joint->y = y;
	joint->z = z;
	joint->phi   = phi;
	joint->theta = theta;
	joint->psi   = psi;

	joint->radius = radius;

	joint->limit_lo = limit_lo;
	joint->limit_hi = limit_hi;
	joint->median = median;
	joint->weight = weight;

    _jointList.push_back(joint);
}

double CKinematics::WeightFunction (double lo, double hi, double q)
{
 	if (lo < hi) {
		if (q < lo) q = lo;
		if (q > hi) q = hi;

		// w = 2(q - lo)/(hi - lo) -1		// 회전각의 범위 [lo, hi]을 [-1, 1]로 스케일 변환한 q의 위치를 계산
		// w = w^32	// U자 모양의 곡선으로 바꾼다.
		double w = 2*(q - lo)/(hi - lo) - 1;
		w *= w; w *= w; w *= w; w *= w; w *= w; 
		return w;
	}
	else {
		return 1.;
	}
}

dMatrix CKinematics::WeightMatrix (unsigned int dof)
{
	dMatrix W(dof, dof);
	W.unit();

/*
	if (_weightCur.size() != dof) {
		_weightCur.resize (dof);
		
		for (unsigned int i=0; i<dof; ++i) {
			JointNullMotion *joint = _jointList[i];
			_weightCur[i] = joint->w;
		}
	}

	_weightPrv = _weightCur;

	for (unsigned int i=0; i<_jointList.size () && i<dof; ++i) {
		JointNullMotion *joint = _jointList[i];

		// Joint의 회전각(q)이 경계값(lo:limits low, hi:limits high) 근처로 갈 때 weight를 감소한다
		_weightCur[i] = joint->w*(1. - WeightFunction(joint->lo, joint->hi, _kin->GetJointAngle(i)));
		// 1 step 이전 weight(_weightPrv)값과 현재 계산된 weight(_weightCur)값을 비교하여
		// weight가 증가할 때는 경계 근처로 가는 경우이므로 weight 값을 그대로 두고
		// weight가 감소할 때는 경계에서 빠져나오는 경우이므로 weight 값을 최고값으로 한다.
		W(i,i) = (_weightPrv[i] < _weightCur[i]) ? joint->w : _weightCur[i];
	}
*/
    for (unsigned int i=0; i<_jointList.size() && i<dof; ++i) {
		JointInfo *joint = _jointList[i];
		W(i,i) = joint->weight;
	}
	return W;
}

dVector CKinematics::NullSpaceVector()
{
	dVector h(_jointList.size ());

    for (unsigned int i=0; i<_jointList.size(); ++i) {
		JointInfo *joint = _jointList[i];

		// Joint의 회전각(q)이 기저값(basis: 사용자가 설정한 로봇의 기본 자세) 근처로 수렴하도록 한다
		h[i] = joint->median - joint->q;
	}

	return h;
}

void CKinematics::ClampMag(dVector &w, double maxMag)
{
    double mag = Norm2(w);
	
	if (maxMag < mag){
		w *= maxMag/mag;
	}
}

dVector CKinematics::SolveJTR(double minAlpha, double clampTask)
{
    Forward();
    dMatrix J = Jacobian();
    dVector de = Error();

    ClampMag(de, clampTask);

	// Compute alpha constant as quotient of 2 inner products as suggested by
	// Buss & Kim, to minimze de after caller updates theta
	dVector JJtde = (J * ~J) * de;
    double alpha = Dot(de, JJtde)/Dot(JJtde, JJtde);

	if (alpha < minAlpha) {
		alpha = minAlpha;
	}

	return alpha * ~J * de;
}

dVector CKinematics::SolveDLS(double dlsLambda, double clampTask, double clampNull)
{
    Forward();
    dMatrix J = Jacobian();
    dVector de = Error();

    ClampMag(de, clampTask);

	dMatrix invJ;

	// Moore-Penrose pseudo-inverse:
	// case m < n: invA = ~A * !(A * ~A)
	if (J.rowno() <= J.colno()) {
		dMatrix I(J.rowno(), J.rowno()); 
		I.unit();
		// Damped least-square method
		invJ = ~J * !( J*~J + dlsLambda*dlsLambda*I );
	}
	// case m > n: invA = !(~A * A) * ~A
	else {
		dMatrix I(J.colno(), J.colno()); 
		I.unit();
		// Damped least-square method
		invJ = !( ~J*J + dlsLambda*dlsLambda*I ) * ~J;
	}

	dVector dq = invJ * de;

	if (0. < clampNull) {
        dVector null = NullSpaceVector();

        ClampMag(null, clampNull);

		dMatrix I(J.colno(), J.colno());
		I.unit();

		dq += (I - invJ*J)*null;
	}

	return dq;
}

dVector CKinematics::GetJointAngle()
{
	dVector ang(_jointList.size());

	for (unsigned int i=0; i<_jointList.size(); ++i) {
		JointInfo *joint = _jointList[i];
		ang[i] = joint->q;
	}
	return ang;
}

void CKinematics::SetJointAngle(dVector &q)
{
	for (unsigned int i = 0; i<_jointList.size() && i<q.size(); ++i) {
		JointInfo *joint = _jointList[i];
		joint->q = q[i];
		if (joint->limit_lo < joint->limit_hi) {
			joint->q = Limit(joint->q, joint->limit_lo, joint->limit_hi);
		}
	}
}

void CKinematics::Apply(dVector &dq)
{
    for (unsigned int i=0; i<_jointList.size() && i<dq.size(); ++i) {
		JointInfo *joint = _jointList[i];
		joint->q += dq[i];
		if (joint->limit_lo < joint->limit_hi) {
			joint->q = Limit (joint->q, joint->limit_lo, joint->limit_hi);
		}
	}
}

