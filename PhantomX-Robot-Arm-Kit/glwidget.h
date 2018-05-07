#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QDebug>

#if defined(_WIN32) || defined(_WIN64)
#include <gl/glu.h>
#elif defined(__APPLE__)
#include <opengl/glu.h>
#endif

#include "opengl/oglDef.h"
#include "kinematics/kinematics.h"
#include "kinematics/posOriInverse.h"

#include "matrix/mathDef.h"
#include "matrix/transformation.h"
#include "opengl/oglObjects.h"

class GLWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit GLWidget(QWidget *parent = 0);

    void setPath(std::vector<std::valarray<double>> *path);
    void setNode(std::vector<std::valarray<double>> *node);
    void setKinematics(CKinematics *kinematics);
    void setJointAngle(dVector &q);

    void mouseMoveEvent(QMouseEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void wheelEvent(QWheelEvent *event);

    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

private:

    // 아래 변수들로부터 로봇을 바라보는 눈의 위치와 방향을 설정한다.
    // 눈의 위치(_eyePos)로부터 물체의 중심(_centerPos)을 바라보는데 수평(_angleHor)과
    // 수직(_angleVer)로 물체를 회전한다.
    CPoint3d _eyePos;
    CPoint3d _centerPos;
    double _angleHor;
    double _angleVer;

    // Perspective projection 설정
    double _fovAngle;
    int _glnWidth;
    int _glnHeight;

    // Mouse 설정
    QPointF _mouseDownPoint;

    // kinematics
    CKinematics *_kinematics;
    std::vector<std::valarray<double>> *_node;
    std::vector<std::valarray<double>> *_path;

    void setViewport();

    void transformAxis(dMatrix A);
    void drawRevLink(double x, double y, double z, double radius);

    void drawFixedJoint(JointInfo *joint);
    void drawRevoluteJoint(JointInfo *joint);
    void drawPrismaticJoint(JointInfo *joint);

    void renderPath(std::vector<std::valarray<double>> *path);
    void renderTarget();
    void renderJoint();
};

#endif // GLWIDGET_H
