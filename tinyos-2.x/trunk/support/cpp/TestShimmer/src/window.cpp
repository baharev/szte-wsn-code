/****************************************************************************
**
** Copyright (C) 2010 Nokia Corporation and/or its subsidiary(-ies).
** All rights reserved.
** Contact: Nokia Corporation (qt-info@nokia.com)
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL$
** Commercial Usage
** Licensees holding valid Qt Commercial licenses may use this file in
** accordance with the Qt Commercial License Agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and Nokia.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU Lesser General Public License version 2.1 requirements
** will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** In addition, as a special exception, Nokia gives you certain additional
** rights.  These rights are described in the Nokia Qt LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3.0 as published by the Free Software
** Foundation and appearing in the file LICENSE.GPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU General Public License version 3.0 requirements will be
** met: http://www.gnu.org/copyleft/gpl.html.
**
** If you have questions regarding the use of this file, please contact
** Nokia at qt-info@nokia.com.
** $QT_END_LICENSE$
**
****************************************************************************/

#include <QtGui>

#include "GLWidget.h"
#include "window.h"
#include "MatrixVector.hpp"
#include "Application.h"

Window::Window(Application &app) : application(app)
{
    QGridLayout *mainLayout = new QGridLayout;

    for (int i = 0; i < NumRows; ++i) {
        for (int j = 0; j < NumColumns; ++j) {
            QColor clearColor;
            clearColor.setHsv(0, 255, 63);

            glWidgets[i][j] = new GLWidget(0, 0);
            glWidgets[i][j]->setClearColor(clearColor);
            glWidgets[i][j]->rotate(+42 * 16, +42 * 16, -21 * 16);
            mainLayout->addWidget(glWidgets[i][j], i, j);

            connect(glWidgets[i][j], SIGNAL(clicked()),
                    this, SLOT(setCurrentGlWidget()));
        }
    }
    setLayout(mainLayout);

    currentGlWidget = glWidgets[0][0];
//    counter = 0;
//
//    QTimer *timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(rotateToNext()));
//    timer->start(200);
//
//    setWindowTitle(tr("Textures"));
}

void Window::setCurrentGlWidget()
{
    currentGlWidget = qobject_cast<GLWidget *>(sender());
}

void Window::rotateToNext()
{

    using namespace gyro;

    enum { X, Y, Z };
    int sample_index;

    vector3 euler = application.dataRecorder.euler_angle_deg(sample_index);

    //currentGlWidget->rotate(euler[X], euler[Y], euler[Z]);

//    //  Rotation around the Z-axis
//    static const int euler[][3] = {
//        { 0, 0,  0 },
//        { 0, 0,  5 },
//        { 0, 0, 10 },
//        { 0, 0, 15 },
//        { 0, 0, 10 },
//        { 0, 0,  5 },
//        { 0, 0,  0 }
//    };
//
//    //  Rotation around the Y-axis
//    static const int euler[][3] = {
//        { 0,  0, 0 },
//        { 0,  5, 0 },
//        { 0, 10, 0 },
//        { 0, 15, 0 },
//        { 0, 10, 0 },
//        { 0,  5, 0 },
//        { 0,  0, 0 }
//    };

    //  Rotation around the X-axis
//    static const int euler[][3] = {
//        {  0, 0, 0 },
//        {  5, 0, 0 },
//        { 10, 0, 0 },
//        { 15, 0, 0 },
//        { 10, 0, 0 },
//        {  5, 0, 0 },
//        {  0, 0, 0 }
//    };

//    const int n = sizeof(euler)/(3*sizeof(int));

//    counter = (++counter)%n;

//    if (currentGlWidget)
//        currentGlWidget->rotate(euler[counter][X], euler[counter][Y], euler[counter][Z]);
}
