#include "glwidget.h"

GLWidget::GLWidget(QWidget *parent) :
    QGLWidget(parent) {

    _eyePos = CPoint3d(1.1, -0.03, 0.4);
    _centerPos = CPoint3d(0, 0, 0.14);
    _angleHor = -35.0;
    _angleVer = 10.5;
    _fovAngle = 45.0;
}

void GLWidget::initializeGL() {

    glClearColor(240.0f,240.0f,240.0f, 1.0);
    glClearDepth(1.0);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    // 조명 설정
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);

    float AmbientColor[]    = { 0.0f, 0.1f, 0.2f, 0.0f };
    float DiffuseColor[]    = { 0.5f, 0.5f, 0.5f, 0.0f };
    float SpecularColor[]   = { 0.5f, 0.5f, 0.5f, 0.0f };
    float Position[]        = { 100.0f, 100.0f, -400.0f, 1.0f };

    glLightfv(GL_LIGHT1, GL_AMBIENT, AmbientColor);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, DiffuseColor);
    glLightfv(GL_LIGHT1, GL_SPECULAR, SpecularColor);
    glLightfv(GL_LIGHT1, GL_POSITION, Position);

    // 재질의 속성 설정
    glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
}

void GLWidget::paintGL() {

    // 그림 그리기
    glMatrixMode(GL_MODELVIEW);    // Select The Modelview Matrix
    glLoadIdentity();              // Reset The Modelview Matrix

    // 그림 지우기
    // Clear The Screen And The Depth Buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    setViewport();
    oglPlane(5.0, 0.5);

    renderTarget();
    renderJoint();

    glFinish();
}

void GLWidget::resizeGL(int width, int height) {

    if (width < 1) width = 1;
    if (height < 1) height = 1;

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);		// Select The Projection Matrix
    glLoadIdentity();					// Reset The Projection Matrix

    _glnWidth = width;
    _glnHeight = height;

    gluPerspective(_fovAngle, (double)_glnWidth/_glnHeight, 0.1, 1000000.0);
}

void GLWidget::mouseMoveEvent(QMouseEvent *event) {

    if(event->buttons() == Qt::LeftButton) {
        _angleHor+=(event->localPos().x()-_mouseDownPoint.x())/3.6;
        _angleVer+=(event->localPos().y()-_mouseDownPoint.y())/3.6;
    } else if(event->buttons() == Qt::RightButton) {
        _centerPos.y -= (event->localPos().x()-_mouseDownPoint.x())/100.0;
        _centerPos.z += (event->localPos().y()-_mouseDownPoint.y())/100.0;
    }

    _mouseDownPoint=event->localPos();

    updateGL();
}

void GLWidget::mousePressEvent(QMouseEvent *event) {


    if(event->buttons() != Qt::MiddleButton) {
        _mouseDownPoint = event->localPos();
    } else {
        _eyePos = CPoint3d(1.1, -0.03, 0.4);
        _centerPos = CPoint3d(0, 0, 0.14);
        _angleHor = -35.0;
        _angleVer = 10.5;
        _fovAngle = 45.0;
        updateGL();
    }
}

void GLWidget::wheelEvent(QWheelEvent *event) {

    _eyePos.x -= _centerPos.x;
    _eyePos.y -= _centerPos.y;
    _eyePos.z -= _centerPos.z;

    _eyePos.x *= (event->delta() < 0) ? 0.9 : 1.1;
    _eyePos.y *= (event->delta() < 0) ? 0.9 : 1.1;
    _eyePos.z *= (event->delta() < 0) ? 0.9 : 1.1;

    _eyePos.x += _centerPos.x;
    _eyePos.y += _centerPos.y;
    _eyePos.z += _centerPos.z;

    updateGL();
}

void GLWidget::setKinematics(CKinematics *kinematics) {
    _kinematics = kinematics;
}

void GLWidget::setJointAngle(dVector &q)
{
    _kinematics->SetJointAngle(q);
}

void GLWidget::setViewport() {
    // 보는 시각 설정
    gluLookAt( _eyePos.x, _eyePos.y, _eyePos.z, _centerPos.x, _centerPos.y, _centerPos.z, 0,0,1 );

    // 좌표계 회전
    glRotated( _angleVer, 0,1,0 );
    glRotated( _angleHor, 0,0,1 );
}

void GLWidget::transformAxis(dMatrix A) {
    double m[16];

    m[0] = A(0,0);  m[4] = A(0,1);  m[8] = A(0,2);  m[12] = A(0,3);
    m[1] = A(1,0);  m[5] = A(1,1);  m[9] = A(1,2);  m[13] = A(1,3);
    m[2] = A(2,0);  m[6] = A(2,1);  m[10]= A(2,2);  m[14] = A(2,3);
    m[3] = 0.0;     m[7] = 0.0;     m[11]= 0.0;     m[15] = 1.0;

    glMultMatrixd(m);
}

void GLWidget::renderTarget () {
    dVector desired = _kinematics->GetDesired ();

    glPushMatrix();

    glTranslated(desired[0], desired[1], desired[2]);
    glRotated (_RAD2DEG*desired[5], 0, 0, 1);
    glRotated (_RAD2DEG*desired[4], 0, 1, 0);
    glRotated (_RAD2DEG*desired[3], 1, 0, 0);

    glColor3d(1.0, 0., 0.);
    oglBox (0.06, 0.06, 0.06);

    oglCoordinate (0.3);

    glPopMatrix();
}

void GLWidget::drawRevLink (double x, double y, double z, double radius) {
    glPushMatrix();

    double rz = _RAD2DEG*atan2(y, x);
    double ry = 90. - _RAD2DEG*atan2(z, sqrt(x*x + y*y));
    double height = sqrt (x*x + y*y + z*z);

    glRotated( rz, 0, 0, 1);
    glRotated( ry, 0, 1, 0);
    glTranslated (0, 0, height/2.);

    oglCylinder (height, radius);

    glPopMatrix();
}

void GLWidget::drawFixedJoint(JointInfo *joint) {
    glColor3d (0.7, 0.7, 0.7);
    drawRevLink (joint->x, joint->y, joint->z, joint->radius);
}

void GLWidget::drawRevoluteJoint(JointInfo *joint) {
    glPushMatrix();

    if      (joint->axis == 0) glRotated( 90., 0,1,0 );
    else if (joint->axis == 1) glRotated(-90., 1,0,0 );

    glColor3d(0.5, 0.5, 0.5);
    oglCylinder (joint->radius*2.5, joint->radius*1.3);

    glPopMatrix();

    glPushMatrix();

    transformAxis(joint->TransformationMatrixQ());

    glColor3d(0.7, 0.7, 0.7);
    drawRevLink (joint->x, joint->y, joint->z, joint->radius);

    glPopMatrix();
}

void GLWidget::drawPrismaticJoint(JointInfo *joint) {
    glPushMatrix();

    if      (joint->axis == 0) glRotated( 90., 0,1,0 );
    else if (joint->axis == 1) glRotated(-90., 1,0,0 );

    glColor3d(0.5, 0.5, 0.5);
    oglBox (joint->radius*2.5, joint->radius*2.5, joint->radius*2.5);
    glTranslated (0, 0, joint->q/2.);

    glColor3d(0.8, 0.8, 0.8);
    oglBox (joint->radius*1.5, joint->radius*1.5, joint->q);

    glPopMatrix();

    glPushMatrix();

    transformAxis(joint->TransformationMatrixQ());

    glColor3d(0.7, 0.7, 0.7);
    drawRevLink (joint->x, joint->y, joint->z, joint->radius);

    glPopMatrix();
}

void GLWidget::renderJoint() {
    vector<JointInfo *> &jointList = _kinematics->GetJointList ();

    for (vector<JointInfo *>::iterator it = jointList.begin (); it != jointList.end (); ++it) {
        JointInfo *joint = *it;

        glColor3d(0.7, 0.7, 0.7);

        if (joint->type == FIXED_JOINT) {
            drawFixedJoint (joint);
        }
        else if (joint->type == REVOLUTE_JOINT) {
            drawRevoluteJoint (joint);
        }
        else if (joint->type == PRISMATIC_JOINT) {
            drawPrismaticJoint (joint);
        }

        transformAxis(joint->TransformationMatrix());
    }

    oglCoordinate (0.3);
}
