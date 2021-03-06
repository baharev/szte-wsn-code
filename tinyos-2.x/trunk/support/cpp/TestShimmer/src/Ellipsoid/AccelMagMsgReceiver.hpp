/* Copyright (c) 2011, University of Szeged
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
*      Author: Ali Baharev
*/

#ifndef ACCELMAGMSGRECEIVER_HPP
#define ACCELMAGMSGRECEIVER_HPP

#include <map>
#include <QObject>
#include "AccelMagSample.hpp"

class QTextStream;

class ActiveMessage;
class CalibrationMatrices;

class AccelMagMsgReceiver : public QObject {

    Q_OBJECT

public:

    AccelMagMsgReceiver();

    bool resetScaleOffset(int moteID);

    bool updateScaleOffset(int moteID, const gyro::vector3& accScl,  const gyro::vector3& accOff,
                                       const gyro::vector3& magnScl, const gyro::vector3& magnOff);

    ~AccelMagMsgReceiver();

signals:

    void newSample(const AccelMagSample sample);

public slots:

    void onReceiveMessage(const ActiveMessage& msg);

private:

    AccelMagMsgReceiver(const AccelMagMsgReceiver& );
    AccelMagMsgReceiver& operator=(const AccelMagMsgReceiver& );

    void loadCalibrationMatrices();

    void loadFile(QTextStream& in);

    const gyro::matrix3 loadMatrix(QTextStream& in) const;

    const gyro::vector3 loadVector(QTextStream& in) const;

    void insertCalibrationMatrices(int moteID, const CalibrationMatrices& M);

    bool computeCalibratedVectors(int moteID, gyro::vector3& accel, gyro::vector3& magn) const;

    std::map<int,CalibrationMatrices>* const calibrationMatrices;

};

#endif // ACCELMAGMSGRECEIVER_HPP
