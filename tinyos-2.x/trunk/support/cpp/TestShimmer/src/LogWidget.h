/** Copyright (c) 2010, 2011, University of Szeged
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
* Author: Péter Ruzicska
*/

#ifndef LOGWIDGET_H
#define LOGWIDGET_H

#include <QWidget>
#include <QTableWidgetItem>
#include "StateColor.hpp"

class QMessageBox;
class Application;

namespace Ui{
    class LogWidget;
}

class LogWidget : public QWidget {

    Q_OBJECT

public:

    LogWidget(QWidget *parent, Application &app);
    ~LogWidget();

    enum TimeMode { SET, SETNOTEDITABLE }; // FIXME SETNOTEDITABLE is never used
    enum Mode { NORMAL, INSERT };
    enum Type { RECORDSTART, RECORDEND, MOTIONSTART, MOTIONEND , TEXT};

    enum Column {
        GOTO = 0,
        STATUS = 1,
        TIME = 2,
        TYPE = 3,
        ENTRY = 4,
        DEL = 5
    };

    enum Status { UNKNOWN, OK, FAILED, EMPTY };

private slots:

    void on_entryLine_returnPressed();
    void on_recStartButton_clicked();
    void on_recEndButton_clicked();
    void on_motionStartButton_clicked();
    void on_motionEndButton_clicked();
    void on_loadButton_clicked();
    void on_saveButton_clicked();
    void on_clearButton_clicked();
    void on_checkButton_clicked();

    void ShowContextMenu(const QPoint& pos);
    void on_log_cellClicked(int,int);
    void on_log_cellChanged(int,int);
    void stateColor(StateColor color);
    void solverFinished(bool error, const QString message);
    void on_log_cellDoubleClicked(int row, int column);

private:

    void onGoto(int);
    void onDelRow(int);

    void createItems(int at, Mode, Type, Status); // FIXME Apart from one function call at==-1
    void createItems(int at, Type, TimeMode, Status, QString txt, QString time);
    void createItem(QString txt, int row, Column, bool editable);
    void createItem(QString txt, int row, Column, bool editable, QIcon);
    int createRow(int row);
    void createGoto(int row, Type);
    void createStatus(int row, Status);
    void createTime(int row);
    void createTime(int row, Mode);
    void createTime(int row, QString time, TimeMode);
    void createType(int row, Type);
    void createEntry(int row, QString txt);
    void createEntry(int row ,Mode, Type);
    void createDel(int row, Type);

    int findMotionStart(int);
    int findMotionEnd(int);

    bool isRecordStart(int) const;
    bool isRecordEnd(int) const;
    bool isMotionStart(int) const;
    bool isMotionEnd(int) const;

    void init();
    void entryLineInit();

    void saveLog(const QString&);
    void loadLog(const QString&);
    void csvToLog(const QString&);

    //=======================
    void startChecking();
    void checkNextMotion();
    void finishedChecking();

    void startSolver();
    void markAsFailed();
    void motionOK();

    int findNextMot();
    int findMotStart(int from) const;
    int findMotEnd(int from) const;

    const QTime motionStart() const;
    const QTime motionEnd() const;
    const QTime recStart() const;
    const QTime timeInRow(int row) const;
    bool isMotionTooShort() const;
    bool isAlreadyPassed(const int row) const;

    int rowCount() const;
    int recLengthInSec() const;

    const QTableWidget& tableWidget() const;
    const QTableWidgetItem& item(int row, Column col) const;
    QTableWidgetItem& item(int row, Column col);
    void setText(int row, Column col, const char text[]);
    void setIcon(int row, Column col, const QIcon& icon);
    void writeToConsole(const QString& msg) const;
    const QString atRow() const;
    //=======================

    Ui::LogWidget *ui;
    Application &application;
    bool inEditing;

    //=======================
    QMessageBox* blockingBox;
    int startAt;
    int endAt;
};

#endif // LOGWIDGET_H
