/** Copyright (c) 2010, University of Szeged
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

#include "LogWidget_editable.h"
#include "ui_LogWidget_editable.h"
#include "Application.h"
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QPushButton>
#include <QDateTime>
#include <QDateTimeEdit>
#include <QMessageBox>

LogWidget_editable::LogWidget_editable(QWidget *parent, Application &app) :
        QWidget(parent),
        ui(new Ui::LogWidget_editable),
        application(app)
{
    ui->setupUi(this);

    signalMapper = new QSignalMapper(this);

    ui->log->setRowCount(0);
    ui->log->horizontalHeader()->resizeSection(0, 40);
    ui->log->horizontalHeader()->resizeSection(1, 110);
    ui->log->horizontalHeader()->resizeSection(2, 70);
    ui->log->horizontalHeader()->setResizeMode(3, QHeaderView::Stretch);
    ui->log->horizontalHeader()->resizeSection(4, 40);

    //ui->log->setEditTriggers(QAbstractItemView::NoEditTriggers);

    ui->entryLine->setFocus();

    connect(signalMapper, SIGNAL(mapped(int)),this, SLOT(onDelRow(int)));
}

LogWidget_editable::~LogWidget_editable()
{

}


void LogWidget_editable::on_entryLine_returnPressed()
{
    if(!ui->entryLine->text().isNull()){
        createItem(ui->entryLine->text());
    }
}

void LogWidget_editable::createItem(QString text)
{
    int row = ui->log->rowCount();
    ui->log->insertRow(row);

    QDateTimeEdit *dateEdit = new QDateTimeEdit(QDate::currentDate());
    dateEdit->setMinimumDate(QDate::currentDate().addDays(-365));
    dateEdit->setMaximumDate(QDate::currentDate().addDays(365));
    dateEdit->setCalendarPopup(true);
    //dateEdit->setReadOnly(true);
    dateEdit->setDisplayFormat("yyyy.MM.dd");
    ui->log->setCellWidget(row, 1, dateEdit);
    //ui->log->cellWidget(row, 1)->setEnabled(false);

    QPalette pal = dateEdit->palette();
    pal.setColor(QPalette::Disabled, QPalette::Text, pal.color(QPalette::Active, QPalette::Text));
    pal.setColor(QPalette::Disabled, QPalette::Base, pal.color(QPalette::Active, QPalette::Base));
    dateEdit->setPalette(pal);

    QPushButton* but = new QPushButton(QIcon(":/icons/back-arrow.png"),"",this);
    //but->setMaximumSize(40,20);
    ui->log->setCellWidget(row,0,but);

    QDateTimeEdit *time = new QDateTimeEdit(QTime::currentTime());
    //time->setReadOnly(true);
    ui->log->setCellWidget(row, 2, time);
    //ui->log->cellWidget(row, 2)->setEnabled(false);

    pal = time->palette();
    pal.setColor(QPalette::Disabled, QPalette::Text, pal.color(QPalette::Active, QPalette::Text));
    pal.setColor(QPalette::Disabled, QPalette::Base, pal.color(QPalette::Active, QPalette::Base));
    time->setPalette(pal);

    QTableWidgetItem* item = new QTableWidgetItem(text,1);
    ui->log->setItem(row,3,item);

    QPushButton* del = new QPushButton(QIcon(":/icons/Delete.png"),"",this);
    del->setMaximumSize(20,20);
    ui->log->setCellWidget(row,4,del);
    signalMapper->setMapping(del, row);

    connect(del, SIGNAL(clicked()), signalMapper, SLOT (map()));

    ui->entryLine->clear();
    ui->entryLine->setFocus();
}

void LogWidget_editable::setTableEditable(bool isEditable)
{
    if(isEditable){
        ui->log->setEditTriggers(QAbstractItemView::DoubleClicked);
        for(int i = 0; i < ui->log->rowCount(); i++){
            ui->log->cellWidget(i,1)->setEnabled(true);
            ui->log->cellWidget(i,2)->setEnabled(true);
        }
    } else {
        ui->log->setEditTriggers(QAbstractItemView::NoEditTriggers);
        for(int i = 0; i < ui->log->rowCount(); i++){
            ui->log->cellWidget(i,1)->setEnabled(false);
            ui->log->cellWidget(i,2)->setEnabled(false);
        }
    }
}

void LogWidget_editable::on_recStartButton_clicked()
{
    createItem(QString::fromUtf8("Rec start"));
}

void LogWidget_editable::on_recEndButton_clicked()
{
    createItem(QString::fromUtf8("Rec end"));
}

void LogWidget_editable::on_motionStartButton_clicked()
{
    createItem(QString::fromUtf8("Motion start"));
}

void LogWidget_editable::on_motionEndButton_clicked()
{
    createItem(QString::fromUtf8("Motion end"));
}

void LogWidget_editable::onDelRow(int row)
{
    QMessageBox msgBox;
    msgBox.setInformativeText("Are you sure you want to delete this row?");
    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Cancel);
    int ret = msgBox.exec();
    if(ret == QMessageBox::Ok)
        ui->log->removeRow(row);
}