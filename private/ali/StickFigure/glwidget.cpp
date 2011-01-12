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
}

GLWidget::GLWidget(QWidget *parent, QGLWidget *shareWidget)
    : QGLWidget(parent, shareWidget)
{
    for (int i=0;i<16; ++i)
        rotmat[i] = (GLfloat) 0.0;

    rotmat[M11] = rotmat[M22] = rotmat[M33] = rotmat[M44] = (GLfloat) 1.0;

    rotmat[M34] = (GLfloat) -10.0;
}

GLWidget::~GLWidget()
{
}

QSize GLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
    return QSize(200, 200);
}

void GLWidget::rotate(double mat[9])
{

    rotmat[M11] = (GLfloat) mat[R11];
    rotmat[M21] = (GLfloat) mat[R21];
    rotmat[M31] = (GLfloat) mat[R31];

    rotmat[M12] = (GLfloat) mat[R12];
    rotmat[M22] = (GLfloat) mat[R22];
    rotmat[M32] = (GLfloat) mat[R32];

    rotmat[M13] = (GLfloat) mat[R13];
    rotmat[M23] = (GLfloat) mat[R23];
    rotmat[M33] = (GLfloat) mat[R33];

    updateGL();
}

void GLWidget::initializeGL()
{
    glClearColor(0.0, 0.0, 0.0, 0.0);
}

void clearAll() {

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f (1.0, 1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();
}

void settings() {

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    glLineWidth(2.0);

    glPolygonMode(GL_FRONT, GL_FILL);
    glPolygonMode(GL_BACK, GL_LINE);
}

void drawArm() {

    static const double h = std::sqrt(3.0)/5.0;

    glBegin(GL_LINES);
        glVertex3d(0.0, 0.0, 0.0);
        glVertex3d(0.0, 2.0, 0.0);
    glEnd();

    glRotated(-10.0, 0.0, 1.0, 0.0);
    glRotated( 30.0, 0.0, 0.0, 1.0);
    glRotated( 40.0, 0.0, 1.0, 0.0);

    glBegin(GL_TRIANGLES);
       glVertex3d(0.0,-1.6, 0.0);
       glVertex3d(0.0,-2.0, 0.0);
       glVertex3d(  h,-1.8, 0.0);
    glEnd();

    glBegin(GL_LINES);
        glVertex3d(0.0,-2.0, 0.0);
        glVertex3d(0.0, 0.0, 0.0);
    glEnd();
}

void GLWidget::paintGL()
{
    qDebug() << "paintGL()";

    clearAll();

    glTranslated(0.0, 0.0, -5.0);

    settings();

    glPushMatrix();
        glTranslated(-5.0, 0.0, 0.0);
        drawArm();
    glPopMatrix();

    glPushMatrix();
        glRotated( 90.0, 1.0, 0.0, 0.0);
        drawArm();
    glPopMatrix();

    glPushMatrix();
        glTranslated( 5.0, 0.0, 0.0);
        glRotated(-90.0, 0.0, 1.0, 0.0);
        drawArm();
    glPopMatrix();
}

void GLWidget::resizeGL(int width, int height)
{
    qDebug() << "resizeGL()";

    int side = qMin(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-7, +7, -7, +7, 2.0, 8.0);
    glMatrixMode(GL_MODELVIEW);
}
