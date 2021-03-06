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
* Author: Mikl?s Mar?ti
* Author: P?ter Ruzicska
*/

#include "GraphWidget.h"
#include "ui_GraphWidget.h"
#include <qfiledialog.h>
#include "QtDebug"
#include "Graph.h"
#include <QtGui>
#include "DataPlot.h"


GraphWidget::GraphWidget(QWidget *parent, Application &app) :
    QWidget(parent),
    ui(new Ui::GraphWidget),
    application(app)
{
    ui->setupUi(this);
    //QGridLayout *mainLayout = new QGridLayout;

    graphs << "X angle"<<"Y angle"<<"Z angle"<<"XY angle"<<"YZ angle"<<"ZX angle";
    ui->xComboBox->addItems(graphs);
    ui->yComboBox->addItems(graphs);

    NumRows = NumColumns = 0;
    //connect(currentGraph, SIGNAL(clicked()), this, SLOT(setCurrentGraphWidget()));

    //setLayout(mainLayout);

}

GraphWidget::~GraphWidget()
{
    delete ui;
}

void GraphWidget::changeEvent(QEvent *e)
{
    QWidget::changeEvent(e);
    switch (e->type())
    {
    case QEvent::LanguageChange:
            ui->retranslateUi(this);
            break;
    default:
            break;
    }
}

void GraphWidget::on_addButton_clicked()
{
    if(NumColumns == 0 && NumRows == 0){
        NumRows++; NumColumns++;
    } else if(NumColumns >= 3){
        NumRows++;
    } else { NumColumns++; }

    for (int i = 0; i < NumRows; ++i) {
        for (int j = 0; j < NumColumns; ++j) {
            scrollAreas[i][j] = new PlotScrollArea(this);
            graphWidgets[i][j] = new Graph(scrollAreas[i][j], application);


            ui->mainLayout->addWidget(scrollAreas[i][j],i,j);


            //mainLayout->addWidget(graphWidgets[i][j], i, j);
            //ui->mainLayout->addWidget(graphWidgets[i][j],i,j,0);
            //currentGraph = graphWidgets[0][0];
            //ui->mainLayout->addWidget(currentGraph);

            //connect(graphWidgets[i][j], SIGNAL(clicked()), this, SLOT(setCurrentGraphWidget()));
        }
    }

    currentGraph = graphWidgets[0][0];
}

void GraphWidget::on_xComboBox_currentIndexChanged()
{
    if(ui->xComboBox->currentIndex() == 0){

    } else if(ui->xComboBox->currentIndex() == 1){

    }
}

void GraphWidget::on_yComboBox_currentIndexChanged()
{
    if(ui->yComboBox->currentIndex() == 0){

    } else if(ui->yComboBox->currentIndex() == 1){
        scrollAreas[0][0] = new PlotScrollArea(this);
        graphWidgets[0][0] = new Graph(scrollAreas[0][0], application);


        ui->mainLayout->addWidget(scrollAreas[0][0],0,0);
    }
}
