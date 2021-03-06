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
#include "ArmWidget.hpp"

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

    enum {
        SOLID_DISK,
        SILHOUETTE,
        LIST_LENGTH
    };

    const int LINE_WIDTH = 3;
}

ArmWidget* ArmWidget::right() {

    return new ArmWidget(AnimationElbowFlexType::right());
}

ArmWidget* ArmWidget::left() {

    return new ArmWidget(AnimationElbowFlexType::left());
}

ArmWidget::ArmWidget(AnimationElbowFlexType sign) : type(sign)
{
    for (int i=0;i<16; ++i) {

        foreArmMat[i] = upperArmMat[i] = (GLfloat) 0.0;
    }

     foreArmMat[M11] =  foreArmMat[M22] =  foreArmMat[M33] =  foreArmMat[M44] = (GLfloat) 1.0;
    upperArmMat[M11] = upperArmMat[M22] = upperArmMat[M33] = upperArmMat[M44] = (GLfloat) 1.0;

    foreArmRefHeading = 0.0;

    labelsAndTable.resize(SIZE_OF_TEXT);
}

ArmWidget::~ArmWidget() {

}

QSize ArmWidget::minimumSizeHint() const {

    return QSize(300, 300);
}

QSize ArmWidget::sizeHint() const {

    return QSize(750, 750);
}

// TODO Can we pass a member function to GLU?
extern void APIENTRY errorCallback();

void ArmWidget::headSilhouette(GLUquadricObj* qobj) {

    gluQuadricDrawStyle(qobj, GLU_SILHOUETTE);

    gluQuadricNormals(qobj, GLU_NONE);

    glNewList(list+SILHOUETTE, GL_COMPILE);

        gluDisk(qobj, 0.0, 0.5, 32, 4);

    glEndList();
}

void ArmWidget::headSolid(GLUquadricObj* qobj) {

    gluQuadricDrawStyle(qobj, GLU_FILL);

    glNewList(list+SOLID_DISK, GL_COMPILE);

        glColor3f (0.0, 0.0, 0.0);

        gluDisk(qobj, 0.0, 0.5, 32, 4);

        glColor3f (1.0, 1.0, 1.0);

    glEndList();
}

void ArmWidget::initializeGL() {

    glClearColor(0.0, 0.0, 0.0, 0.0);

    list = glGenLists(LIST_LENGTH);

    GLUquadricObj* qobj = gluNewQuadric();

    gluQuadricCallback(qobj, GLU_ERROR, errorCallback);

    headSilhouette(qobj);

    headSolid(qobj);

    gluDeleteQuadric(qobj);
}

void ArmWidget::reset() {

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f (1.0, 1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);

    glLoadIdentity();
}

void ArmWidget::setState() {

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

    glEnable(GL_DEPTH_TEST);

    glLineWidth(LINE_WIDTH);
}

void ArmWidget::shoulder() {

    glBegin(GL_LINES);
        glVertex3d(0.0, 2.0, 0.0);
        glVertex3d(0.0, 2.0, 2.0*type.sign);
    glEnd();
}

void ArmWidget::neck() {

    glBegin(GL_LINES);
        glVertex3d(0.0, 2.0, 1.0*type.sign);
        glVertex3d(0.0, 2.5, 1.0*type.sign);
    glEnd();
}

void ArmWidget::stillUpperArm() {

    glBegin(GL_LINES);
        glVertex3d(0.0, 2.0, 2.0*type.sign);
        glVertex3d(0.0, 0.0, 2.0*type.sign);
    glEnd();
}

void ArmWidget::body() {

    glLineWidth(1.0);

    glBegin(GL_LINES);
        glVertex3d(0.0, 2.0, 1.0*type.sign);
        glVertex3d(0.0,-0.4, 1.0*type.sign);
    glEnd();

    glLineWidth(LINE_WIDTH);
}

void ArmWidget::upperArm() {

    glPushMatrix();

        glTranslated(0.0, 2.0, 0.0);

        glMultMatrixf(upperArmMat);

        glBegin(GL_LINES);
            glVertex3d(0.0, 0.0, 0.0);
            glVertex3d(0.0,-2.0, 0.0);
        glEnd();

    glPopMatrix();
}

void ArmWidget::moveToElbow() {

    const double LENGTH = -2;

    const double x = upperArmMat[M12]*LENGTH;
    const double y = upperArmMat[M22]*LENGTH - LENGTH;
    const double z = upperArmMat[M32]*LENGTH;

    glTranslated(x, y, z);
}

void ArmWidget::elbow() {

    glPointSize(8.0);

    glBegin(GL_POINTS);
        glVertex3d(0.0, 0.0, 0.0);
    glEnd();

    glPointSize(1.0);
}

void ArmWidget::applyForeArmRotation() {

    glMultMatrixf(foreArmMat);
}

void ArmWidget::foreArm() {

    glBegin(GL_LINES);
        glVertex3d(0.0,-1.6, 0.0);
        glVertex3d(0.0, 0.0, 0.0);
    glEnd();
}

void ArmWidget::hand() {

    glPolygonMode(GL_FRONT, type.handFront);
    glPolygonMode(GL_BACK,  type.handBack);

    glBegin(GL_POLYGON);
        glVertex3d(-0.15,-1.6, 0.0);
        glVertex3d(-0.15,-2.0, 0.0);
        glVertex3d( 0.15,-2.0, 0.0);
        glVertex3d( 0.15,-1.6, 0.0);
    glEnd();

    glBegin(GL_LINES);
        glVertex3d(0.15, -1.7, 0.0);
        glVertex3d(0.3 , -1.7, 0.0);
    glEnd();

}

void ArmWidget::drawIrrelevantParts() {

    glLineWidth(1.0);

    shoulder();

    neck();

    stillUpperArm();

    glLineWidth(LINE_WIDTH);
}

void ArmWidget::drawMovingArm() {

    rotateToUpperArmRef();

    upperArm();

    moveToElbow();

    elbow();

    rotateToForeArmRef();

    applyForeArmRotation();

    foreArm();

    hand();
}

void ArmWidget::drawLinearParts() {

    drawIrrelevantParts();

    drawMovingArm();
}

void ArmWidget::setCameraPosition() {

    glTranslated(0.0, 0.0, -5.0);
}

void ArmWidget::sideHead() {

    glPushMatrix();

        glTranslated(0.0, 3.0, 0.0);

        glPointSize(8.0);

        glBegin(GL_POINTS);
            glVertex3d(0.4, 0.1, 0.0);
        glEnd();

        glPointSize(1.0);

        glCallList(list+SILHOUETTE);


    glPopMatrix();
}

void ArmWidget::sideView() {

    glPushMatrix();

        sideHead();

        drawLinearParts();
    glPopMatrix();
}

const char* ArmWidget::text(TEXT_POSITION pos) const {

    return labelsAndTable.at(pos).c_str();
}

void ArmWidget::writeData() {

    glPushMatrix();

    //(x, y, z): " << rotmat[M11] << ", " << rotmat[M21] << ", " << rotmat[M31];
    // Flex: -90...270; Sup: -180...180; Dev: -90...90

    renderText(-1.0, 3.65, 0.0, text(FLEX_LABEL));

    renderText( 2.0, 3.65, 0.0, text(FRAME_LABEL));

    renderText( 5.0, 3.65, 0.0, text(SUP_LABEL));

    renderText(-1.0, -2.85, 0.0, text(DEV_LABEL));

    renderText(2.5, -2.5, 0.0, "           begin / end / min / max / range  deg");

    renderText(2.5, -3.0, 0.0, text(FLEX_LINE));

    renderText(2.5, -3.5, 0.0, text(EXT_LINE));

    renderText(2.5, -4.0, 0.0, text(SUP_LINE));

    renderText(2.5, -4.5, 0.0, text(PRON_LINE));

    renderText(2.5, -5.0, 0.0, text(LAT_LINE));

    renderText(2.5, -5.5, 0.0, text(MED_LINE));

    glPopMatrix();
}

void ArmWidget::planHead() {

    glPushMatrix();

        glTranslated(0.0, 1.0*(-type.sign), 2.5);

        glPointSize(8.0);

        glBegin(GL_POINTS);
            glVertex3d(0.4, 0.1, 0.0);
            glVertex3d(0.4,-0.1, 0.0);
        glEnd();

        glPointSize(1.0);

        glCallList(list+SILHOUETTE);

        glTranslated(0.0, 0.0,-0.1);

        glCallList(list+SOLID_DISK);

    glPopMatrix();
}

void ArmWidget::planView() {

    glPushMatrix();

        glTranslated(0.0, type.planShift, 0.0);

        planHead();

        glRotated( 90.0, 1.0, 0.0, 0.0);

        drawLinearParts();
    glPopMatrix();
}

void ArmWidget::frontHead() {

    glPushMatrix();

        glTranslated(1.0*(-type.sign), 3.0, 0.0);

        glPointSize(8.0);

        glBegin(GL_POINTS);
            glVertex3d(-0.1, 0.1, 0.0);
            glVertex3d( 0.1, 0.1, 0.0);
        glEnd();

        glPointSize(1.0);

        glCallList(list+SILHOUETTE);

    glPopMatrix();
}

void ArmWidget::frontView() {

    glPushMatrix();

        glTranslated(type.frontShift, 0.0, 0.0);

        frontHead();

        glRotated(-90.0, 0.0, 1.0, 0.0);

        body();

        drawLinearParts();
    glPopMatrix();
}

void ArmWidget::paintGL() {

    reset();

    setCameraPosition();

    setState();

    sideView();

    writeData();

    planView();

    frontView();
}

void ArmWidget::resizeGL(int width, int height) {

    const double left  = -2.5;
    const double right =  7.5;
    const double bottom = -7.5;
    const double up     =  4;

    const double w = right-left;
    const double h = up-bottom;

    const double unit = qMin(width/w, height/h);

    glViewport((width-w*unit)/2, (height-h*unit)/2, w*unit, h*unit);

    glMatrixMode(GL_PROJECTION);

    glLoadIdentity();

    glOrtho(left, right, bottom, up, 2.0, 8.0);

    glMatrixMode(GL_MODELVIEW);
}

void ArmWidget::mousePressEvent(QMouseEvent * /* event */)
{
    emit clicked();
}

void ArmWidget::display(const std::map<int,gyro::matrix3>& matrices, const std::vector<std::string>& text) {

    if (matrices.empty()) {

        return;
    }

    typedef std::map<int,gyro::matrix3>::const_iterator itr;

    itr i = matrices.begin();

    updateMatrix(foreArmMat, i->second);

    ++i;

    if (i!=matrices.end()) {

        updateMatrix(upperArmMat, i->second);
    }

    labelsAndTable = text;

    updateGL();
}

void ArmWidget::setReference(const std::vector<double>& headings) {

    foreArmRefHeading = headings.at(0);

    upperArmRefHeading = headings.at(1);
}

void ArmWidget::rotateToUpperArmRef() {

    glRotated(upperArmRefHeading, 0.0, 1.0, 0.0);
}

void ArmWidget::rotateToForeArmRef() {

    glRotated(foreArmRefHeading-upperArmRefHeading, 0.0, 1.0, 0.0);
}

void ArmWidget::updateMatrix(GLfloat rotMat[16], const gyro::matrix3& rotationMatrix) {

    double mat[9];

    rotationMatrix.copy_to(mat);

    rotMat[M21] = (GLfloat)-mat[R11];
    rotMat[M22] = (GLfloat)-mat[R12];
    rotMat[M23] = (GLfloat)-mat[R13];

    rotMat[M11] = (GLfloat) mat[R21];
    rotMat[M12] = (GLfloat) mat[R22];
    rotMat[M13] = (GLfloat) mat[R23];

    rotMat[M31] = (GLfloat)-mat[R31];
    rotMat[M32] = (GLfloat)-mat[R32];
    rotMat[M33] = (GLfloat)-mat[R33];
}
