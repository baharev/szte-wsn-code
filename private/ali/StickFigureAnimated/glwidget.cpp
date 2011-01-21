/* Copyright (c) 2011 University of Szeged
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* - Neither the name of University of Szeged nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Author: Ali Baharev
*/

#include <QtGui>
#include <QtOpenGL>
#include <QTextStream>
#include <QDebug>
#include <cmath>
#include "glwidget.hpp"

namespace {

    enum {
            R11, R12, R13,
            R21, R22, R23,
            R31, R32, R33
    };

    enum {
            M11 = 0, M12 = 4, M13 =  8, M14 = 12,
            M21 = 1, M22 = 5, M23 =  9, M24 = 13,
            M31 = 2, M32 = 6, M33 = 10, M34 = 14,
            M41 = 3, M42 = 7, M43 = 11, M44 = 15
    };

    const double RAD2DEG = 57.2957795131;
    const double PI_HALF = 1.57079632679;
}

GLWidget::GLWidget(QWidget *parent, QGLWidget *shareWidget)
    : QGLWidget(parent, shareWidget)
{
    for (int i=0;i<16; ++i)
        rotmat[i] = (GLfloat) 0.0;

    rotmat[M11] = rotmat[M22] = rotmat[M33] = rotmat[M44] = (GLfloat) 1.0;
}

GLWidget::~GLWidget() {

}

QSize GLWidget::minimumSizeHint() const {

    return QSize(50, 50);
}

QSize GLWidget::sizeHint() const {

    return QSize(200, 200);
}

void GLWidget::rotate(const double mat[9]) {

    rotmat[M31] = (GLfloat) mat[R11];
    rotmat[M21] = (GLfloat) mat[R31];
    rotmat[M11] = (GLfloat) mat[R21];

    rotmat[M32] = (GLfloat) mat[R12];
    rotmat[M22] = (GLfloat) mat[R32];
    rotmat[M12] = (GLfloat) mat[R22];

    rotmat[M33] = (GLfloat) mat[R13];
    rotmat[M23] = (GLfloat) mat[R33];
    rotmat[M13] = (GLfloat) mat[R23];

    updateGL();
}

// TODO Can we pass a member function to GLU?
void errorCallback() {

    const GLubyte* estring = gluErrorString(glGetError());

    printf("OpenGL Error: %s\n", estring);

    exit(0);
}

void GLWidget::initializeGL() {

    glClearColor(0.0, 0.0, 0.0, 0.0);

    list = glGenLists(1);

    GLUquadricObj* qobj = gluNewQuadric();

    gluQuadricCallback(qobj, GLU_ERROR, errorCallback);

    gluQuadricDrawStyle(qobj, GLU_SILHOUETTE);

    gluQuadricNormals(qobj, GLU_NONE);

    glNewList(list, GL_COMPILE);

        gluDisk(qobj, 0.0, 0.5, 32, 4);

    glEndList();
}

void GLWidget::reset() {

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f (1.0, 1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();
}

void GLWidget::setState() {

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

    glLineWidth(2.0);
}

void GLWidget::upperArm() {

    glBegin(GL_LINES);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(0.0, 2.0, 0.0);
    glEnd();
}

void GLWidget::elbow() {

    glPointSize(8.0);

    glBegin(GL_POINTS);
        glVertex3d(0.0, 0.0, 0.0);
    glEnd();

    glPointSize(1.0);
}

void GLWidget::rotateForeArm() {

    glMultMatrixf(rotmat);
}

void GLWidget::foreArm() {

    glBegin(GL_LINES);
        glVertex3d(2.0, 0.0, 0.0);
        glVertex3d(0.0, 0.0, 0.0);
    glEnd();
}

void GLWidget::hand() {

    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK, GL_LINE);

    static const double h = std::sqrt(3.0)/5.0;

    glBegin(GL_TRIANGLES);
       glVertex3d(1.6, 0.0, 0.0);
       glVertex3d(2.0, 0.0, 0.0);
       glVertex3d(1.8,   h, 0.0);
    glEnd();
}


void GLWidget::drawArm() {

    upperArm();

    elbow();

    rotateForeArm();

    foreArm();

    hand();
}

void GLWidget::setCameraPosition() {

    glTranslated(0.0, 0.0, -5.0);
}

void GLWidget::sideHead() {

    glPushMatrix();

        glTranslated(0.0, 3.0, 0.0);

        glPointSize(8.0);

        glBegin(GL_POINTS);
            glVertex3d(0.4, 0.1, 0.0);
        glEnd();

        glPointSize(1.0);

        glCallList(list);


    glPopMatrix();
}

void GLWidget::sideView() {

    glPushMatrix();

        glTranslated(-5.0, 0.0, 0.0);

        sideHead();

        drawArm();
    glPopMatrix();
}

// FIXME Computatation should be moved to datareader.cpp, clean-up the mess
void GLWidget::writeAngles() {

    glPushMatrix();

    QString text;

    QTextStream ts(&text);

    ts.setRealNumberNotation(QTextStream::FixedNotation);

    ts.setRealNumberPrecision(2);

    //ts << "(x, y, z): " << rotmat[M11] << ", " << rotmat[M21] << ", " << rotmat[M31] << "  ";

    // -90...270
    ts << "Flex " << (atan2(rotmat[M21], rotmat[M11])+PI_HALF)*RAD2DEG;
    ts << " (" << extrema[FLEX_MIN] << "/" << extrema[FLEX_MAX] << "/" << extrema[FLEX_MAX]-extrema[FLEX_MIN] << ")";
    ts << " deg   ";

    // -180...180   //  -90...270   (atan2(-rotmat[M33],rotmat[M32])+PI_HALF)*RAD2DEG
    ts << "Sup " << (atan2(rotmat[M32],rotmat[M33]))*RAD2DEG;
    ts << " (" << extrema[SUP_MIN] << "/" << extrema[SUP_MAX] << "/" << extrema[SUP_MAX]-extrema[SUP_MIN] << ")";
    ts << " (" << extrema[PRON_MIN] << "/" << extrema[PRON_MAX] << "/" << extrema[PRON_MAX]-extrema[PRON_MIN] << ")";
    ts << " deg   ";

    // -90...90
    ts << "Dev " << (acos(rotmat[M31])-PI_HALF)*RAD2DEG;
    ts << " (" << extrema[LAT_DEV_MIN] << "/" << extrema[LAT_DEV_MAX] << "/" << extrema[LAT_DEV_MAX]-extrema[LAT_DEV_MIN] << ")";
    ts << " (" << extrema[MED_DEV_MIN] << "/" << extrema[MED_DEV_MAX] << "/" << extrema[MED_DEV_MAX]-extrema[MED_DEV_MIN] << ")";
    ts << " deg";

    ts.flush();

    renderText(-6.5, 3.65, 0.0, text);

    glPopMatrix();
}

void GLWidget::planHead() {

    glPushMatrix();

        glTranslated(0.0, 1.0, 0.0);

        glPointSize(8.0);

        glBegin(GL_POINTS);
            glVertex3d(0.4, 0.1, 0.0);
            glVertex3d(0.4,-0.1, 0.0);
        glEnd();

        glPointSize(1.0);

        glCallList(list);

    glPopMatrix();
}

void GLWidget::planView() {

    planHead();

    glPushMatrix();

        glRotated( 90.0, 1.0, 0.0, 0.0);

        drawArm();
    glPopMatrix();
}

void GLWidget::frontHead() {

    glPushMatrix();

        glTranslated(1.0, 3.0, 0.0);

        glPointSize(8.0);

        glBegin(GL_POINTS);
            glVertex3d(-0.1, 0.1, 0.0);
            glVertex3d( 0.1, 0.1, 0.0);
        glEnd();

        glPointSize(1.0);

        glCallList(list);

    glPopMatrix();
}

void GLWidget::frontView() {

    glPushMatrix();

        glTranslated( 5.0, 0.0, 0.0);

        frontHead();

        glRotated(-90.0, 0.0, 1.0, 0.0);

        drawArm();
    glPopMatrix();
}

void GLWidget::paintGL() {

    reset();

    setCameraPosition();

    setState();

    sideView();

    writeAngles();

    planView();

    frontView();
}

void GLWidget::resizeGL(int width, int height) {

    double unit = qMin(width/15.0, height/6.5);

    glViewport((width-15*unit)/2, (height-6.5*unit)/2, 15*unit, 6.5*unit);

    glMatrixMode(GL_PROJECTION);

    glLoadIdentity();

    glOrtho(-7.5, +7.5, -2.5, +4.0, 2.0, 8.0);

    glMatrixMode(GL_MODELVIEW);
}

void GLWidget::mousePressEvent(QMouseEvent * /* event */)
{
    emit clicked();
}

void GLWidget::set_extrema(const double m[]) {

    for (int i=0; i<SIZE; ++i) {
        extrema[i] = m[i];
    }
}
