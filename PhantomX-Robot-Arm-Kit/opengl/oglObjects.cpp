//////////////////////////////////////////////////////
// OpenGL files for Robotics applications
//
// Copyright (c) 2002-2010. All Rights Reserved.
// Division of Applied Robot Technology, KITECH
// Web: http://www.orobot.net
// Written by KwangWoong Yang<page365@gmail.com>
//

#if defined(_WIN32) || defined(_WIN64)
    #include <gl/glu.h>
#elif defined(__APPLE__)
    #include <opengl/glu.h>
#elif defined(__linux__)
    #include <GL/gl.h>
    #include <GL/glu.h>
#endif




#include "matrix/mathDef.h"
#include "oglObjects.h"

void oglPlane(double size, double stride) {
	size /= 2.;
    int n = (int)(size / stride);

	glDisable(GL_LIGHTING);

	glColor3d(0.5, 0.5, 0.5);

    for (int i = -n ; i <= n; ++i) {
        oglLine (i * stride, -size, 0, i * stride, (i == 0) ? 0 : size, 0);
        oglLine (-size, i * stride, 0, (i == 0) ? 0 : size, i * stride, 0);
	}
	// x-axis
	glColor3d(1., 0., 0.);
	oglLine (0, 0, 0, size, 0, 0);

	// y-axis
	glColor3d(0., 1., 0.);
	oglLine (0, 0, 0, 0, size, 0);

	// z-axis
	glColor3d(0., 0., 1.);
	oglLine (0, 0, 0, 0, 0, size);

	glEnable(GL_LIGHTING);
}

void oglCoordinate(double size) {
	glDisable(GL_LIGHTING);

	// The Z-axis
	glColor3d(0., 0., 1.);
	oglLine (0., 0., 0., 0., 0., size);

	// The Y-axis
	glColor3d(0., 1., 0.);
	oglLine(0., 0., 0., 0., size, 0.);

	// The X-axis
	glColor3d(1., 0., 0.);
	oglLine(0., 0., 0., size, 0., 0.);

	glEnable(GL_LIGHTING);
}

void oglLine (double sx, double sy, double sz, double ex, double ey, double ez) {
	glBegin(GL_LINES);
	glVertex3d(sx, sy, sz);
	glVertex3d(ex, ey, ez);
	glEnd();
}

void oglLineStrip(vector <CPoint3d> &line) {
	glBegin(GL_LINE_STRIP);
    for(unsigned int i = 0; i < line.size(); i++) {
		glVertex3d(line[i].x, line[i].y, line[i].z);
	}
	glEnd();
}

void oglSphere (double radius) {
	int slices = 15;
	GLUquadricObj *obj = gluNewQuadric();

	gluSphere(obj, radius, slices, slices );

	gluDeleteQuadric(obj);
}

void NormalVector (double nv[3], double p1[3], double p2[3], double p3[3]) {
    double v1[3] = {
        p2[0] - p1[0],
        p2[1] - p1[1],
        p2[2] - p1[2],
	};
    double v2[3] = {
        p3[0] - p1[0],
        p3[1] - p1[1],
        p3[2] - p1[2],
	};
	double v3[3] = {
        v1[1] *v2[2] - v1[2] *v2[1],
        v1[2] *v2[0] - v1[0] *v2[2],
        v1[0] *v2[1] - v1[1] *v2[0],
	};

    double l = sqrt (v3[0] * v3[0] + v3[1] * v3[1] + v3[2] * v3[2]);

    nv[0] = v3[0] / l;
    nv[1] = v3[1] / l;
    nv[2] = v3[2] / l;
}

void oglSphere(double rx, double ry, double rz) {
	const int lats = 25;
	const int longs = 25;

    double pt[lats + 1][longs + 1][3];
    double nv[lats + 1][longs + 1][3];

	for (int i = 0; i <= lats; i++) {
        double lat = -M_PI / 2. + M_PI * (double)i / lats;
        double z  = rz * sin(lat);
		double zr =  cos(lat);

		for (int j = 0; j <= longs; j++) {
            double lng = 2. * M_PI * (double)j / longs;
            pt[i][j][0] = zr * rx * cos(lng);
            pt[i][j][1] = zr * ry * sin(lng);
			pt[i][j][2] = z;
		}
	}

	for (int i = 1; i < lats; i++) {
		int im = i - 1;
		int ip = i + 1;
		for (int j = 0; j <= longs; j++) {
            int jm = (longs + j - 1) % longs;
            int jp = (longs + j + 1) % longs;
			NormalVector (nv[i][j], pt[im][jm], pt[im][jp], pt[ip][jm]);
		}
	}

	{
        glBegin(GL_TRIANGLE_FAN);
		glNormal3d(0., 0., -1.);
		glVertex3d(0., 0., -rz);
		int i = 1;
		for (int j = 0; j <= longs; j++) {
			glNormal3d(nv[i][j][0], nv[i][j][1], nv[i][j][2]);
			glVertex3d(pt[i][j][0], pt[i][j][1], pt[i][j][2]);
		}
		glEnd();
	}
	for (int i = 2; i < lats; i++) {
		glBegin(GL_QUAD_STRIP);
		for (int j = 0; j <= longs; j++) {
            glNormal3d(nv[i - 1][j][0], nv[i - 1][j][1], nv[i - 1][j][2]);
            glVertex3d(pt[i - 1][j][0], pt[i - 1][j][1], pt[i - 1][j][2]);
            glNormal3d(nv[i - 0][j][0], nv[i - 0][j][1], nv[i - 0][j][2]);
            glVertex3d(pt[i - 0][j][0], pt[i - 0][j][1], pt[i - 0][j][2]);
		}
		glEnd();
	}
	{
        glBegin(GL_TRIANGLE_FAN);
		glNormal3d(0., 0., +1.);
		glVertex3d(0., 0., +rz);
        int i = lats - 1;
		for (int j = 0; j <= longs; j++) {
			glNormal3d(nv[i][j][0], nv[i][j][1], nv[i][j][2]);
			glVertex3d(pt[i][j][0], pt[i][j][1], pt[i][j][2]);
		}
		glEnd();
	}
}

void oglCapsule (double height, double radius) {
	height /= 2.;

	int slices = 25;
	GLUquadricObj *obj = gluNewQuadric();

	glTranslated(0., 0., -height);
	oglSphere (radius);

	gluQuadricOrientation(obj, GLU_OUTSIDE);
    gluCylinder(obj, radius, radius, height * 2, slices, 1);

    glTranslated(0., 0., height * 2);
	oglSphere (radius);

	glTranslated(0., 0., -height);

	gluDeleteQuadric(obj);
}

void oglCylinder (double height, double radius) {
	height /= 2.;

	int slices = 25;
	GLUquadricObj *obj = gluNewQuadric();

	glTranslated(0., 0., -height);
	gluQuadricOrientation(obj, GLU_INSIDE);
	gluDisk(obj, 0, radius, slices, 1);

	gluQuadricOrientation(obj, GLU_OUTSIDE);
    gluCylinder(obj, radius, radius, height * 2, slices, 1);

    glTranslated(0., 0., height * 2);
	gluDisk(obj, 0., radius, slices, 1);

	glTranslated(0., 0., -height);

	gluDeleteQuadric(obj);
}

void oglCylinder(double height, double baseRadius, double topRadius, double xyRatio) {
	height /= 2.;

	const int longs = 25;

    double pt[2][longs + 1][3];
    double nv[1][longs + 1][3];

	for(int j = 0; j <= longs; j++) {
        double lng = 2. * M_PI * (double)j / longs;
        pt[0][j][0] = baseRadius * cos(lng);
        pt[0][j][1] = baseRadius * xyRatio * sin(lng);
		pt[0][j][2] = -height;

        pt[1][j][0] = topRadius * cos(lng);
        pt[1][j][1] = topRadius * xyRatio * sin(lng);
		pt[1][j][2] = +height;
	}

	for (int j = 0; j <= longs; j++) {
		nv[0][j][0] = 0.;
		nv[0][j][1] = 0.;
		nv[0][j][2] = 1.;
	}

	for (int j = 0; j <= longs; j++) {
        int jm = (longs + j - 1) % longs;
        int jp = (longs + j + 1) % longs;
		NormalVector (nv[0][j], pt[0][jm], pt[0][jp], pt[1][jm]);
	}

	{
        glBegin(GL_TRIANGLE_FAN);
		glNormal3d(0., 0., -1.);
		glVertex3d(0., 0., -height);
		for (int j = 0; j <= longs; j++) {
			glNormal3d(0., 0., -1.);
			glVertex3d(pt[0][j][0], pt[0][j][1], pt[0][j][2]);
		}
		glEnd();
	}
	{
		glBegin(GL_QUAD_STRIP);
		for (int j = 0; j <= longs; j++) {
			glNormal3d(nv[0][j][0], nv[0][j][1], nv[0][j][2]);
			glVertex3d(pt[0][j][0], pt[0][j][1], pt[0][j][2]);
			glNormal3d(nv[0][j][0], nv[0][j][1], nv[0][j][2]);
			glVertex3d(pt[1][j][0], pt[1][j][1], pt[1][j][2]);
		}
		glEnd();
	}
	{
        glBegin(GL_TRIANGLE_FAN);
		glNormal3d(0., 0., +1.);
		glVertex3d(0., 0., +height);
		for (int j = 0; j <= longs; j++) {
			glNormal3d(0., 0., +1.);
			glVertex3d(pt[1][j][0], pt[1][j][1], pt[1][j][2]);
		}
		glEnd();
	}
}

void oglBox (double dx, double dy, double dz) {
	dx /= 2.;
	dy /= 2.;
	dz /= 2.;

	glBegin(GL_QUADS);
	glNormal3d(0., 0., 1.);
    glTexCoord2f(  0.0f,   0.0f);
    glVertex3d( dx,  dy,  dz);
    glTexCoord2f(100.0f,   0.0f);
    glVertex3d( dx, -dy,  dz);
    glTexCoord2f(100.0f, 100.0f);
    glVertex3d(-dx, -dy,  dz);
    glTexCoord2f(  0.0f, 100.0f);
    glVertex3d(-dx,  dy,  dz);

	glNormal3d(0., 0., -1.);
    glTexCoord2f(  0.0f,   0.0f);
    glVertex3d(-dx,  dy, -dz);
    glTexCoord2f(100.0f,   0.0f);
    glVertex3d(-dx, -dy, -dz);
    glTexCoord2f(100.0f, 100.0f);
    glVertex3d( dx, -dy, -dz);
    glTexCoord2f(  0.0f, 100.0f);
    glVertex3d( dx,  dy, -dz);

	glNormal3d(1., 0., 0.);
    glTexCoord2f(  0.0f,   0.0f);
    glVertex3d( dx,  dy,  dz);
    glTexCoord2f(100.0f,   0.0f);
    glVertex3d( dx,  dy, -dz);
    glTexCoord2f(100.0f, 100.0f);
    glVertex3d( dx, -dy, -dz);
    glTexCoord2f(  0.0f, 100.0f);
    glVertex3d( dx, -dy,  dz);

	glNormal3d(-1., 0., 0.);
    glTexCoord2f(  0.0f,   0.0f);
    glVertex3d(-dx,  dy,  dz);
    glTexCoord2f(100.0f,   0.0f);
    glVertex3d(-dx,  dy, -dz);
    glTexCoord2f(100.0f, 100.0f);
    glVertex3d(-dx, -dy, -dz);
    glTexCoord2f(  0.0f, 100.0f);
    glVertex3d(-dx, -dy,  dz);

	glNormal3d(0., 1., 0.);
    glTexCoord2f(  0.0f,   0.0f);
    glVertex3d(-dx,  dy,  dz);
    glTexCoord2f(100.0f,   0.0f);
    glVertex3d( dx,  dy,  dz);
    glTexCoord2f(100.0f, 100.0f);
    glVertex3d( dx,  dy, -dz);
    glTexCoord2f(  0.0f, 100.0f);
    glVertex3d(-dx,  dy, -dz);

	glNormal3d(0., -1., 0.);
    glTexCoord2f(  0.0f,   0.0f);
    glVertex3d(-dx, -dy,  dz);
    glTexCoord2f(100.0f,   0.0f);
    glVertex3d( dx, -dy,  dz);
    glTexCoord2f(100.0f, 100.0f);
    glVertex3d( dx, -dy, -dz);
    glTexCoord2f(  0.0f, 100.0f);
    glVertex3d(-dx, -dy, -dz);
	glEnd();
}
