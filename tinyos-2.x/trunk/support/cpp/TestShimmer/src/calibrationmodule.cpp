#include "CalibrationModule.h"
#include "DataRecorder.h"
#include <QtDebug>
#include <qfile.h>
#include "LinearEquations.h"
#include "tnt_math_utils.h"
#include <cmath>

using namespace std;

CalibrationModule::CalibrationModule(Application &app) : application(app)
{
    xMin = 9999; yMin = 9999; zMin = 9999; xGyrMin = 9999; yGyrMin = 9999, zGyrMin = 9999;
    xMax = 0; yMax = 0; zMax = 0; xGyrMax = 0; yGyrMax = 0; zGyrMax = 0;
    xDiff = 0; yDiff = 0; zDiff = 0; xGyrDiff = 0; yGyrDiff = 0; zGyrDiff = 0;
    xAvg = 0.0; yAvg = 0.0; zAvg = 0.0;
    xMinAvg = 9999.99; xMaxAvg = 0.0; yMinAvg = 9999.99; yMaxAvg = 0.0; zMinAvg = 9999.99; zMaxAvg = 0.0;
    xGyrAvg = 0.0; yGyrAvg = 0.0; zGyrAvg = 0.0;
    xMinGyrAvg = 9999.99; xMaxGyrAvg = 0.0; yMinGyrAvg = 9999.99; yMaxGyrAvg = 0.0; zMinGyrAvg = 9999.99; zMaxGyrAvg = 0.0;
    xSum = 0; ySum = 0; zSum = 0;
    xGyrSum = 0; yGyrSum = 0; zGyrSum = 0;
}

CalibrationModule::~CalibrationModule()
{
    clearWindows();
    clearIdleSides();
}

IdleWindow::IdleWindow()
{
    xMin = -1;
    xMax = -1;
    xAvg = -1;
    yMin = -1;
    yMax = -1;
    yAvg = -1;
    zMin = -1;
    zMax = -1;
    zAvg = -1;
}

QString CalibrationModule::Calibrate()
{
    int xAccel,yAccel,zAccel, xGyro, yGyro, zGyro;


    for(int j = 0; j < application.dataRecorder.size()-WINDOW; j+=10) {
        for(int i = j; i < j + WINDOW; i++){
            xAccel = application.dataRecorder.at(i).xAccel;
            yAccel = application.dataRecorder.at(i).yAccel;
            zAccel = application.dataRecorder.at(i).zAccel;
            xGyro  = application.dataRecorder.at(i).xGyro;
            yGyro  = application.dataRecorder.at(i).yGyro;
            zGyro  = application.dataRecorder.at(i).zGyro;

            if(xAccel > xMax) xMax = xAccel; if(xAccel < xMin) xMin = xAccel;
            if(yAccel > yMax) yMax = yAccel; if(yAccel < yMin) yMin = yAccel;
            if(zAccel > zMax) zMax = zAccel; if(zAccel < zMin) zMin = zAccel;

            if(xGyro > xGyrMax) xGyrMax = xGyro; if(xGyro < xGyrMin) xGyrMin = xGyro;
            if(yGyro > yGyrMax) yGyrMax = yGyro; if(yGyro < yGyrMin) yGyrMin = yGyro;
            if(zGyro > zGyrMax) zGyrMax = zGyro; if(zGyro < zGyrMin) zGyrMin = zGyro;

            xSum += xAccel; xGyrSum += xGyro;
            ySum += yAccel; yGyrSum += yGyro;
            zSum += zAccel; zGyrSum += zGyro;
        }
        xAvg = (float) xSum/WINDOW; yAvg = (float) ySum/WINDOW; zAvg = (float) zSum/WINDOW;
        xGyrAvg = (float) xGyrSum/WINDOW; yGyrAvg = (float) yGyrSum/WINDOW; zGyrAvg = (float) zGyrSum/WINDOW;

        if(xAvg > xMaxAvg) xMaxAvg = xAvg; if(xAvg < xMinAvg) xMinAvg = xAvg;
        if(yAvg > yMaxAvg) yMaxAvg = yAvg; if(yAvg < yMinAvg) yMinAvg = yAvg;
        if(zAvg > zMaxAvg) zMaxAvg = zAvg; if(zAvg < zMinAvg) zMinAvg = zAvg;

        if(xGyrAvg > xMaxGyrAvg) xMaxGyrAvg = xGyrAvg; if(xGyrAvg < xMinGyrAvg) xMinGyrAvg = xGyrAvg;
        if(yGyrAvg > yMaxGyrAvg) yMaxGyrAvg = yGyrAvg; if(yGyrAvg < yMinGyrAvg) yMinGyrAvg = yGyrAvg;
        if(zGyrAvg > zMaxGyrAvg) zMaxGyrAvg = zGyrAvg; if(zGyrAvg < zMinGyrAvg) zMinGyrAvg = zGyrAvg;

        xDiff = xMax - xMin; xGyrDiff = xGyrMax - xGyrMin;
        yDiff = yMax - yMin; yGyrDiff = yGyrMax - yGyrMin;
        zDiff = zMax - zMin; zGyrDiff = zGyrMax - zGyrMin;

        if( (xDiff < MAXDIFF) && (yDiff < MAXDIFF) && (zDiff < MAXDIFF) && (xGyrDiff < MAXDIFF) && (yGyrDiff < MAXDIFF) && (zGyrDiff < MAXDIFF) ){
            IdleWindow idleWindow;

            idleWindow.xMax = xMax; idleWindow.xMin = xMin; idleWindow.xAvg = xAvg;
            idleWindow.yMax = yMax; idleWindow.yMin = yMin; idleWindow.yAvg = yAvg;
            idleWindow.zMax = zMax; idleWindow.zMin = zMin; idleWindow.zAvg = zAvg;
            idleWindow.xGyroAvg = xGyrAvg; idleWindow.yGyroAvg = yGyrAvg; idleWindow.zGyroAvg = zGyrAvg;
            idleWindows.append(idleWindow);
        }

        xMin = 9999; yMin = 9999; zMin = 9999; xGyrMin = 9999; yGyrMin = 9999, zGyrMin = 9999;
        xMax = 0; yMax = 0; zMax = 0; xGyrMax = 0; yGyrMax = 0; zGyrMax = 0;
        xDiff = 0; yDiff = 0; zDiff = 0; xGyrDiff = 0; yGyrDiff = 0; zGyrDiff = 0;
        xAvg = 0.0; yAvg = 0.0; zAvg = 0.0;
        xGyrAvg = 0.0; yGyrAvg = 0.0; zGyrAvg = 0.0;
        xSum = 0; ySum = 0; zSum = 0;
        xGyrSum = 0; yGyrSum = 0; zGyrSum = 0;
    }

    if (idleWindows.size() < 6){
        return "Please load a data record with the mote being idle on each side for at least a 2 seconds!";
    } else {
        return Classify();
    }
}

QString CalibrationModule::Classify()
{
    float xTemp,yTemp,zTemp;
    int xPMinDiff = 3*MAXDIFF; int xNMinDiff = 3*MAXDIFF;
    int yPMinDiff = 3*MAXDIFF; int yNMinDiff = 3*MAXDIFF;
    int zPMinDiff = 3*MAXDIFF; int zNMinDiff = 3*MAXDIFF;

    for (int i = 0; i < 6; i++) {
        idleSidesMins[i] = -1;
    }

    for (int i=0; i<idleWindows.size(); i++) {
        xTemp = idleWindows[i].xAvg; yTemp = idleWindows[i].yAvg; zTemp = idleWindows[i].zAvg;
        int actDiff = (idleWindows[i].xMax-idleWindows[i].xMin)+(idleWindows[i].yMax-idleWindows[i].yMin)+(idleWindows[i].zMax-idleWindows[i].zMin);
        if ( (xTemp >= xP) && (xTemp <= xMaxAvg) && (yTemp >= yZ_L) && (yTemp <= yZ_U ) && (zTemp >= zZ_L) && (zTemp <= zZ_U ) ) {
            //+x side
            if ( xPMinDiff > actDiff ) {
                xPMinDiff = actDiff;
                idleSidesMins[0] = i;
            }
        } else if ( (xTemp >= xMinAvg) && (xTemp <= xN) && (yTemp >= yZ_L) && (yTemp <= yZ_U ) && (zTemp >= zZ_L) && (zTemp <= zZ_U ) ) {
            //-x side
            if ( xNMinDiff > actDiff ) {
                xNMinDiff = actDiff;
                idleSidesMins[1] = i;
            }
        } else if ( (xTemp >= xZ_L) && (xTemp <= xZ_U ) && (yTemp >= yP) && (yTemp <= yMaxAvg) && (zTemp >= zZ_L) && (zTemp <= zZ_U ) ) {
            //+y side
            if ( yPMinDiff > actDiff ) {
                yPMinDiff = actDiff;
                idleSidesMins[2] = i;
            }
        } else if ( (xTemp >= xZ_L) && (xTemp <= xZ_U ) && (yTemp >= yMinAvg) && (yTemp <= yN) && (zTemp >= zZ_L) && (zTemp <= zZ_U ) ) {
            //-y side
            if ( yNMinDiff > actDiff ) {
                yNMinDiff = actDiff;
                idleSidesMins[3] = i;
            }
        } else if ( (xTemp >= xZ_L) && (xTemp <= xZ_U ) && (yTemp >= yZ_L) && (yTemp <= yZ_U ) && (zTemp >= zP) && (zTemp <= zMaxAvg) ) {
            //+z side
            if ( zPMinDiff > actDiff ) {
                zPMinDiff = actDiff;
                idleSidesMins[4] = i;
            }
        } else if ( (xTemp >= xZ_L) && (xTemp <= xZ_U ) && (yTemp >= yZ_L) && (yTemp <= yZ_U ) && (zTemp >= zMinAvg) && (zTemp <= zN) ) {
            //-z side
            if ( zNMinDiff > actDiff ) {
                zNMinDiff = actDiff;
                idleSidesMins[5] = i;
            }
        } else {
            //calibration error
            return "Idle window outside boundaries!";
        }
    }
    xMinAvg = 9999.99; xMaxAvg = 0.0; yMinAvg = 9999.99; yMaxAvg = 0.0; zMinAvg = 9999.99; zMaxAvg = 0.0;

    for ( int i = 0; i < 6; i++ ) {
        if ( idleSidesMins[i] == -1 ) {
            return "Calibration is missing the mote being idle on one of its sides! (Please load a record with the mote being idle on each side for at least 2 seconds!)";
        }
    }

    double xGyrAvgSum = 0;
    double yGyrAvgSum = 0;
    double zGyrAvgSum = 0;
    for ( int i = 1; i < 6 ; i++ ) {
        if ( fabs(idleWindows[idleSidesMins[i-1]].xGyroAvg - idleWindows[idleSidesMins[i]].xGyroAvg) > MAXDIFF ) {
            if ( fabs(idleWindows[idleSidesMins[i-1]].yGyroAvg - idleWindows[idleSidesMins[i]].yGyroAvg) > MAXDIFF ) {
                if ( fabs(idleWindows[idleSidesMins[i-1]].zGyroAvg - idleWindows[idleSidesMins[i]].zGyroAvg) > MAXDIFF ) {
                    return "Gyroscope is not idle. Your mote might have been spinning while recording the calibration data. Please ensure your mote is completly idle.";
                }
            }
        } else {
            xGyrAvgSum += idleWindows[idleSidesMins[i-1]].xGyroAvg;
            yGyrAvgSum += idleWindows[idleSidesMins[i-1]].yGyroAvg;
            zGyrAvgSum += idleWindows[idleSidesMins[i-1]].zGyroAvg;
        }
    }
    xGyrAvgSum += idleWindows[idleSidesMins[5]].xGyroAvg;
    yGyrAvgSum += idleWindows[idleSidesMins[5]].yGyroAvg;
    zGyrAvgSum += idleWindows[idleSidesMins[5]].zGyroAvg;

    gyroMinAvgs[0] = xGyrAvgSum / 6.0;
    gyroMinAvgs[1] = yGyrAvgSum / 6.0;
    gyroMinAvgs[2] = zGyrAvgSum / 6.0;

    return LSF();
}

QString CalibrationModule::LSF() {

    LinearEquations linearEquations;

    Equation* equation1 = linearEquations.createEquation();
    equation1->setConstant(GRAV);
    equation1->setCoefficient("a11", idleWindows[idleSidesMins[0]].xAvg);
    equation1->setCoefficient("a12", idleWindows[idleSidesMins[0]].yAvg);
    equation1->setCoefficient("a13", idleWindows[idleSidesMins[0]].zAvg);
    equation1->setCoefficient("a21", 0);
    equation1->setCoefficient("a22", 0);
    equation1->setCoefficient("a23", 0);
    equation1->setCoefficient("a31", 0);
    equation1->setCoefficient("a32", 0);
    equation1->setCoefficient("a33", 0);
    equation1->setCoefficient("b1", 1.0);
    equation1->setCoefficient("b2", 0);
    equation1->setCoefficient("b3", 0);
    linearEquations.addEquation(equation1);

    Equation* equation2 = linearEquations.createEquation();
    equation2->setConstant(0.0);
    equation2->setCoefficient("a11", 0);
    equation2->setCoefficient("a12", 0);
    equation2->setCoefficient("a13", 0);
    equation2->setCoefficient("a21", idleWindows[idleSidesMins[0]].xAvg);
    equation2->setCoefficient("a22", idleWindows[idleSidesMins[0]].yAvg);
    equation2->setCoefficient("a23", idleWindows[idleSidesMins[0]].zAvg);
    equation2->setCoefficient("a31", 0);
    equation2->setCoefficient("a32", 0);
    equation2->setCoefficient("a33", 0);
    equation2->setCoefficient("b1", 0);
    equation2->setCoefficient("b2", 1);
    equation2->setCoefficient("b3", 0);
    linearEquations.addEquation(equation2);

    Equation* equation3 = linearEquations.createEquation();
    equation3->setConstant(0.0);
    equation3->setCoefficient("a11", 0);
    equation3->setCoefficient("a12", 0);
    equation3->setCoefficient("a13", 0);
    equation3->setCoefficient("a21", 0);
    equation3->setCoefficient("a22", 0);
    equation3->setCoefficient("a23", 0);
    equation3->setCoefficient("a31", idleWindows[idleSidesMins[0]].xAvg);
    equation3->setCoefficient("a32", idleWindows[idleSidesMins[0]].yAvg);
    equation3->setCoefficient("a33", idleWindows[idleSidesMins[0]].zAvg);
    equation3->setCoefficient("b1", 0);
    equation3->setCoefficient("b2", 0);
    equation3->setCoefficient("b3", 1);
    linearEquations.addEquation(equation3);

    Equation* equation4 = linearEquations.createEquation();
    equation4->setConstant(-GRAV);
    equation4->setCoefficient("a11", idleWindows[idleSidesMins[1]].xAvg);
    equation4->setCoefficient("a12", idleWindows[idleSidesMins[1]].yAvg);
    equation4->setCoefficient("a13", idleWindows[idleSidesMins[1]].zAvg);
    equation4->setCoefficient("a21", 0);
    equation4->setCoefficient("a22", 0);
    equation4->setCoefficient("a23", 0);
    equation4->setCoefficient("a31", 0);
    equation4->setCoefficient("a32", 0);
    equation4->setCoefficient("a33", 0);
    equation4->setCoefficient("b1", 1.0);
    equation4->setCoefficient("b2", 0);
    equation4->setCoefficient("b3", 0);
    linearEquations.addEquation(equation4);

    Equation* equation5 = linearEquations.createEquation();
    equation5->setConstant(0.0);
    equation5->setCoefficient("a11", 0);
    equation5->setCoefficient("a12", 0);
    equation5->setCoefficient("a13", 0);
    equation5->setCoefficient("a21", idleWindows[idleSidesMins[1]].xAvg);
    equation5->setCoefficient("a22", idleWindows[idleSidesMins[1]].yAvg);
    equation5->setCoefficient("a23", idleWindows[idleSidesMins[1]].zAvg);
    equation5->setCoefficient("a31", 0);
    equation5->setCoefficient("a32", 0);
    equation5->setCoefficient("a33", 0);
    equation5->setCoefficient("b1", 0);
    equation5->setCoefficient("b2", 1);
    equation5->setCoefficient("b3", 0);
    linearEquations.addEquation(equation5);

    Equation* equation6 = linearEquations.createEquation();
    equation6->setConstant(0.0);
    equation6->setCoefficient("a11", 0);
    equation6->setCoefficient("a12", 0);
    equation6->setCoefficient("a13", 0);
    equation6->setCoefficient("a21", 0);
    equation6->setCoefficient("a22", 0);
    equation6->setCoefficient("a23", 0);
    equation6->setCoefficient("a31", idleWindows[idleSidesMins[1]].xAvg);
    equation6->setCoefficient("a32", idleWindows[idleSidesMins[1]].yAvg);
    equation6->setCoefficient("a33", idleWindows[idleSidesMins[1]].zAvg);
    equation6->setCoefficient("b1", 0);
    equation6->setCoefficient("b2", 0);
    equation6->setCoefficient("b3", 1);
    linearEquations.addEquation(equation6);

    Equation* equation7 = linearEquations.createEquation();
    equation7->setConstant(0.0);
    equation7->setCoefficient("a11", idleWindows[idleSidesMins[2]].xAvg);
    equation7->setCoefficient("a12", idleWindows[idleSidesMins[2]].yAvg);
    equation7->setCoefficient("a13", idleWindows[idleSidesMins[2]].zAvg);
    equation7->setCoefficient("a21", 0);
    equation7->setCoefficient("a22", 0);
    equation7->setCoefficient("a23", 0);
    equation7->setCoefficient("a31", 0);
    equation7->setCoefficient("a32", 0);
    equation7->setCoefficient("a33", 0);
    equation7->setCoefficient("b1", 1.0);
    equation7->setCoefficient("b2", 0);
    equation7->setCoefficient("b3", 0);
    linearEquations.addEquation(equation7);

    Equation* equation8 = linearEquations.createEquation();
    equation8->setConstant(GRAV);
    equation8->setCoefficient("a11", 0);
    equation8->setCoefficient("a12", 0);
    equation8->setCoefficient("a13", 0);
    equation8->setCoefficient("a21", idleWindows[idleSidesMins[2]].xAvg);
    equation8->setCoefficient("a22", idleWindows[idleSidesMins[2]].yAvg);
    equation8->setCoefficient("a23", idleWindows[idleSidesMins[2]].zAvg);
    equation8->setCoefficient("a31", 0);
    equation8->setCoefficient("a32", 0);
    equation8->setCoefficient("a33", 0);
    equation8->setCoefficient("b1", 0);
    equation8->setCoefficient("b2", 1);
    equation8->setCoefficient("b3", 0);
    linearEquations.addEquation(equation8);

    Equation* equation9 = linearEquations.createEquation();
    equation9->setConstant(0.0);
    equation9->setCoefficient("a11", 0);
    equation9->setCoefficient("a12", 0);
    equation9->setCoefficient("a13", 0);
    equation9->setCoefficient("a21", 0);
    equation9->setCoefficient("a22", 0);
    equation9->setCoefficient("a23", 0);
    equation9->setCoefficient("a31", idleWindows[idleSidesMins[2]].xAvg);
    equation9->setCoefficient("a32", idleWindows[idleSidesMins[2]].yAvg);
    equation9->setCoefficient("a33", idleWindows[idleSidesMins[2]].zAvg);
    equation9->setCoefficient("b1", 0);
    equation9->setCoefficient("b2", 0);
    equation9->setCoefficient("b3", 1);
    linearEquations.addEquation(equation9);

    Equation* equation10 = linearEquations.createEquation();
    equation10->setConstant(0.0);
    equation10->setCoefficient("a11", idleWindows[idleSidesMins[3]].xAvg);
    equation10->setCoefficient("a12", idleWindows[idleSidesMins[3]].yAvg);
    equation10->setCoefficient("a13", idleWindows[idleSidesMins[3]].zAvg);
    equation10->setCoefficient("a21", 0);
    equation10->setCoefficient("a22", 0);
    equation10->setCoefficient("a23", 0);
    equation10->setCoefficient("a31", 0);
    equation10->setCoefficient("a32", 0);
    equation10->setCoefficient("a33", 0);
    equation10->setCoefficient("b1", 1.0);
    equation10->setCoefficient("b2", 0);
    equation10->setCoefficient("b3", 0);
    linearEquations.addEquation(equation10);

    Equation* equation11 = linearEquations.createEquation();
    equation11->setConstant(-GRAV);
    equation11->setCoefficient("a11", 0);
    equation11->setCoefficient("a12", 0);
    equation11->setCoefficient("a13", 0);
    equation11->setCoefficient("a21", idleWindows[idleSidesMins[3]].xAvg);
    equation11->setCoefficient("a22", idleWindows[idleSidesMins[3]].yAvg);
    equation11->setCoefficient("a23", idleWindows[idleSidesMins[3]].zAvg);
    equation11->setCoefficient("a31", 0);
    equation11->setCoefficient("a32", 0);
    equation11->setCoefficient("a33", 0);
    equation11->setCoefficient("b1", 0);
    equation11->setCoefficient("b2", 1);
    equation11->setCoefficient("b3", 0);
    linearEquations.addEquation(equation11);

    Equation* equation12 = linearEquations.createEquation();
    equation12->setConstant(0.0);
    equation12->setCoefficient("a11", 0);
    equation12->setCoefficient("a12", 0);
    equation12->setCoefficient("a13", 0);
    equation12->setCoefficient("a21", 0);
    equation12->setCoefficient("a22", 0);
    equation12->setCoefficient("a23", 0);
    equation12->setCoefficient("a31", idleWindows[idleSidesMins[3]].xAvg);
    equation12->setCoefficient("a32", idleWindows[idleSidesMins[3]].yAvg);
    equation12->setCoefficient("a33", idleWindows[idleSidesMins[3]].zAvg);
    equation12->setCoefficient("b1", 0);
    equation12->setCoefficient("b2", 0);
    equation12->setCoefficient("b3", 1);
    linearEquations.addEquation(equation12);

    Equation* equation13 = linearEquations.createEquation();
    equation13->setConstant(0.0);
    equation13->setCoefficient("a11", idleWindows[idleSidesMins[4]].xAvg);
    equation13->setCoefficient("a12", idleWindows[idleSidesMins[4]].yAvg);
    equation13->setCoefficient("a13", idleWindows[idleSidesMins[4]].zAvg);
    equation13->setCoefficient("a21", 0);
    equation13->setCoefficient("a22", 0);
    equation13->setCoefficient("a23", 0);
    equation13->setCoefficient("a31", 0);
    equation13->setCoefficient("a32", 0);
    equation13->setCoefficient("a33", 0);
    equation13->setCoefficient("b1", 1.0);
    equation13->setCoefficient("b2", 0);
    equation13->setCoefficient("b3", 0);
    linearEquations.addEquation(equation13);

    Equation* equation14 = linearEquations.createEquation();
    equation14->setConstant(0.0);
    equation14->setCoefficient("a11", 0);
    equation14->setCoefficient("a12", 0);
    equation14->setCoefficient("a13", 0);
    equation14->setCoefficient("a21", idleWindows[idleSidesMins[4]].xAvg);
    equation14->setCoefficient("a22", idleWindows[idleSidesMins[4]].yAvg);
    equation14->setCoefficient("a23", idleWindows[idleSidesMins[4]].zAvg);
    equation14->setCoefficient("a31", 0);
    equation14->setCoefficient("a32", 0);
    equation14->setCoefficient("a33", 0);
    equation14->setCoefficient("b1", 0);
    equation14->setCoefficient("b2", 1);
    equation14->setCoefficient("b3", 0);
    linearEquations.addEquation(equation14);

    Equation* equation15 = linearEquations.createEquation();
    equation15->setConstant(GRAV);
    equation15->setCoefficient("a11", 0);
    equation15->setCoefficient("a12", 0);
    equation15->setCoefficient("a13", 0);
    equation15->setCoefficient("a21", 0);
    equation15->setCoefficient("a22", 0);
    equation15->setCoefficient("a23", 0);
    equation15->setCoefficient("a31", idleWindows[idleSidesMins[4]].xAvg);
    equation15->setCoefficient("a32", idleWindows[idleSidesMins[4]].yAvg);
    equation15->setCoefficient("a33", idleWindows[idleSidesMins[4]].zAvg);
    equation15->setCoefficient("b1", 0);
    equation15->setCoefficient("b2", 0);
    equation15->setCoefficient("b3", 1);
    linearEquations.addEquation(equation15);

    Equation* equation16 = linearEquations.createEquation();
    equation16->setConstant(0.0);
    equation16->setCoefficient("a11", idleWindows[idleSidesMins[5]].xAvg);
    equation16->setCoefficient("a12", idleWindows[idleSidesMins[5]].yAvg);
    equation16->setCoefficient("a13", idleWindows[idleSidesMins[5]].zAvg);
    equation16->setCoefficient("a21", 0);
    equation16->setCoefficient("a22", 0);
    equation16->setCoefficient("a23", 0);
    equation16->setCoefficient("a31", 0);
    equation16->setCoefficient("a32", 0);
    equation16->setCoefficient("a33", 0);
    equation16->setCoefficient("b1", 1.0);
    equation16->setCoefficient("b2", 0);
    equation16->setCoefficient("b3", 0);
    linearEquations.addEquation(equation16);

    Equation* equation17 = linearEquations.createEquation();
    equation17->setConstant(0.0);
    equation17->setCoefficient("a11", 0);
    equation17->setCoefficient("a12", 0);
    equation17->setCoefficient("a13", 0);
    equation17->setCoefficient("a21", idleWindows[idleSidesMins[5]].xAvg);
    equation17->setCoefficient("a22", idleWindows[idleSidesMins[5]].yAvg);
    equation17->setCoefficient("a23", idleWindows[idleSidesMins[5]].zAvg);
    equation17->setCoefficient("a31", 0);
    equation17->setCoefficient("a32", 0);
    equation17->setCoefficient("a33", 0);
    equation17->setCoefficient("b1", 0);
    equation17->setCoefficient("b2", 1);
    equation17->setCoefficient("b3", 0);
    linearEquations.addEquation(equation17);

    Equation* equation18 = linearEquations.createEquation();
    equation18->setConstant(-GRAV);
    equation18->setCoefficient("a11", 0);
    equation18->setCoefficient("a12", 0);
    equation18->setCoefficient("a13", 0);
    equation18->setCoefficient("a21", 0);
    equation18->setCoefficient("a22", 0);
    equation18->setCoefficient("a23", 0);
    equation18->setCoefficient("a31", idleWindows[idleSidesMins[5]].xAvg);
    equation18->setCoefficient("a32", idleWindows[idleSidesMins[5]].yAvg);
    equation18->setCoefficient("a33", idleWindows[idleSidesMins[5]].zAvg);
    equation18->setCoefficient("b1", 0);
    equation18->setCoefficient("b2", 0);
    equation18->setCoefficient("b3", 1);
    linearEquations.addEquation(equation18);

//  linearEquations.printStatistics();

    Solution* solution = linearEquations.solveWithSVD(0.0);
//  solution->print();

    QString returnMessage = "";

    if ( solution->getMaximumError() > 0.1 ) {
        returnMessage = "Maximum Error is too great! ( > 0.1 )  \n";
        return returnMessage;
    } else {
        application.settings.beginWriteArray("calibrationData");
        for (unsigned int i = 0; i < linearEquations.getVariableCount(); i++) {
            application.settings.setArrayIndex(i);
            application.settings.setValue("calibrationData", solution->getValueAt(i));
        }
        application.settings.endArray();

        application.settings.beginWriteArray("gyroAvgsData");
        for (unsigned int i = 0; i < 3; i++) {
            application.settings.setArrayIndex(i);
            application.settings.setValue("gyroAvgsData", gyroMinAvgs[i]);
        }
        application.settings.endArray();

        application.settings.beginReadArray("calibrationData");
        returnMessage.append("\nCalibration Data: \n");

        application.settings.setArrayIndex(0);
        returnMessage.append("a11 ");
        returnMessage.append( application.settings.value("calibrationData").toString() + "\n" );
        calibrationData[0] = application.settings.value("calibrationData").toDouble();

        application.settings.setArrayIndex(1);
        returnMessage.append("a12 ");
        returnMessage.append( application.settings.value("calibrationData").toString() + "\n" );
        calibrationData[1] = application.settings.value("calibrationData").toDouble();

        application.settings.setArrayIndex(2);
        returnMessage.append("a13 ");
        returnMessage.append( application.settings.value("calibrationData").toString() + "\n" );
        calibrationData[2] = application.settings.value("calibrationData").toDouble();

        application.settings.setArrayIndex(3);
        returnMessage.append("a21 ");
        returnMessage.append( application.settings.value("calibrationData").toString() + "\n" );
        calibrationData[3] = application.settings.value("calibrationData").toDouble();

        application.settings.setArrayIndex(4);
        returnMessage.append("a22 ");
        returnMessage.append( application.settings.value("calibrationData").toString() + "\n" );
        calibrationData[4] = application.settings.value("calibrationData").toDouble();

        application.settings.setArrayIndex(5);
        returnMessage.append("a23 ");
        returnMessage.append( application.settings.value("calibrationData").toString() + "\n" );
        calibrationData[5] = application.settings.value("calibrationData").toDouble();

        application.settings.setArrayIndex(6);
        returnMessage.append("a31 ");
        returnMessage.append( application.settings.value("calibrationData").toString() + "\n" );
        calibrationData[6] = application.settings.value("calibrationData").toDouble();

        application.settings.setArrayIndex(7);
        returnMessage.append("a32 ");
        returnMessage.append( application.settings.value("calibrationData").toString() + "\n" );
        calibrationData[7] = application.settings.value("calibrationData").toDouble();

        application.settings.setArrayIndex(8);
        returnMessage.append("a33 ");
        returnMessage.append( application.settings.value("calibrationData").toString() + "\n" );
        calibrationData[8] = application.settings.value("calibrationData").toDouble();

        application.settings.setArrayIndex(9);
        returnMessage.append("b1 ");
        returnMessage.append( application.settings.value("calibrationData").toString() + "\n" );
        calibrationData[9] = application.settings.value("calibrationData").toDouble();

        application.settings.setArrayIndex(10);
        returnMessage.append("b2 ");
        returnMessage.append( application.settings.value("calibrationData").toString() + "\n" );
        calibrationData[10] = application.settings.value("calibrationData").toDouble();

        application.settings.setArrayIndex(11);
        returnMessage.append("b3 ");
        returnMessage.append( application.settings.value("calibrationData").toString() + "\n" );
        calibrationData[11] = application.settings.value("calibrationData").toDouble();

        application.settings.endArray();

        return returnMessage;
    }
}

QString IdleWindow::toString() const
{
    QString s = "Idle Window: \n";

    s += "xAvg:" + QString::number(xAvg)
         + "  yAvg:" + QString::number(yAvg)
         + "  zAvg:" + QString::number(zAvg);

        return s + "\n";
}

void CalibrationModule::clearWindows()
{
        idleWindows.clear();
}

void CalibrationModule::clearIdleSides()
{
    for (int i = 0; i < 6; i++) {
        idleSidesMins[i] = 0;
    }
}

void CalibrationModule::printMatrix1D(TNT::Array1D<double> matrix) {
    std::cout << "\n";
    for (int i = 0; i < matrix.dim1(); i++) {
        std::cout << matrix[i] <<"\n";
    }
    std::cout << "\n" << flush;
}

void CalibrationModule::printMatrix2D(TNT::Array2D<double> matrix) {
    std::cout << "\n";
    for (int i = 0; i < matrix.dim1(); i++) {
        for (int j = 0; j < matrix.dim2(); j++) {
            std::cout << matrix[i][j] <<"\t";
        }
        std::cout << "\n";
    }
    std::cout << flush;
}
