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
#include "glwidget.hpp"
#include "window.hpp"

window::window() : ANIMATION_STEP_MS(20) {

    createGLWidget();

    createSlider();

    createButton();

    createTimer();

    setupLayout();

    setupConnections();

    timerStart();
}

void window::createGLWidget() {

    widget = new GLWidget(this, 0);

    widget->setData("MMtricky2");
}

void window::createSlider() {

    slider = new QSlider(Qt::Horizontal, this);

    slider->setRange(0, widget->numberOfSamples()-1);

    slider->setSingleStep(4);

    slider->setPageStep(205); // FIXME Knows the sampling rate

    slider->setTickInterval(205);

    slider->setTickPosition(QSlider::TicksBothSides);
}

void window::createButton() {

    playButton = new QPushButton("Pause", this);

    int width = playButton->width();

    playButton->setFixedWidth(width);
}

void window::createTimer() {

    timer = new QTimer(this);
}

void window::setupLayout() {

    QHBoxLayout* controls = new QHBoxLayout();

    controls->addWidget(playButton);

    controls->addWidget(slider);

    QGridLayout* mainLayout = new QGridLayout(this);

    mainLayout->addWidget(widget, 0, 0);

    mainLayout->addLayout(controls, 1, 0);

    setLayout(mainLayout);
}

void window::setupConnections() {

    connect(timer, SIGNAL(timeout()), this, SLOT(nextFrame()));

    connect(slider, SIGNAL(valueChanged(int)), this, SLOT(setFrame(int)));

    connect(widget, SIGNAL(clicked()), this, SLOT(toggleAnimationState()));

    connect(playButton, SIGNAL(pressed()), this, SLOT(toggleAnimationState()));
}

void window::nextFrame() {

    slider->triggerAction(QAbstractSlider::SliderSingleStepAdd);
}

void window::setFrame(int pos) {

    if (pos == slider->maximum()) {

        timerStop();
    }

    widget->setFrame(pos);
}

void window::keyPressEvent(QKeyEvent * event) {

    if (event->key() == Qt::Key_Space) {

        toggleAnimationState();
    }
}

void window::toggleAnimationState() {

    if (timer->isActive()) {

        timerStop();
    }
    else {

        timerStart();
    }
}

void window::timerStart() {

    playButton->setText("Pause");

    if (slider->sliderPosition() == slider->maximum()) {

        slider->triggerAction(QAbstractSlider::SliderToMinimum);
    }

    timer->start(ANIMATION_STEP_MS);
}

void window::timerStop() {

    playButton->setText("Play");

    timer->stop();
}
