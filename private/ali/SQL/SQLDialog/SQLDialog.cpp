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

#include <QDateEdit>
#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QHeaderView>
#include <QMessageBox>
#include <QString>
#include <QSqlError>
#include <QSqlQuery>
#include <QSqlQueryModel>
#include <QTableView>
#include "SQLDialog.hpp"

SQLDialog::SQLDialog() {

    setupDBConnection();

    setWindowModality(Qt::ApplicationModal);

    QHBoxLayout* line = new QHBoxLayout;

    line->addWidget(new QLabel("Name: "));

    QLineEdit* nameInput = new QLineEdit;

    connect(nameInput, SIGNAL(textChanged(QString)), SLOT(nameEdited(QString)));

    line->addWidget(nameInput);

    line->addWidget(new QLabel("Born: "));

    QDateEdit* date = new QDateEdit(QDate::currentDate());

    date->setDisplayFormat("MMM d yyyy");

    line->addWidget(date);

    model = new QSqlQueryModel;

    model->setQuery(*query);

    model->setHeaderData(0, Qt::Horizontal, tr("Name"));

    model->setHeaderData(1, Qt::Horizontal, tr("Born"));

    QTableView *view = new QTableView;

    view->setModel(model);

    view->verticalHeader()->hide();

    view->horizontalHeader()->setStretchLastSection(true);

    view->resizeColumnsToContents();

    QVBoxLayout* layout = new QVBoxLayout;

    layout->addLayout(line);

    layout->addWidget(view);

    setLayout(layout);
}

void SQLDialog::setupDBConnection() {

    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");

    db.setDatabaseName("../records.sqlite");

    if (!db.open()) {
        QMessageBox::critical(this, "Failed to open the DB",
                              "Failed to open the database of the records!");
    }

    query = new QSqlQuery("SELECT name, birthday FROM person");
}

void SQLDialog::nameEdited(const QString& name) {

    QString query = "SELECT name, birthday FROM person WHERE name LIKE '"+name+"%' ORDER BY name, birthday";

    model->setQuery(query);
}

QSize SQLDialog::minimumSizeHint() const {

    return QSize(300, 300);
}

QSize SQLDialog::sizeHint() const {

    return QSize(750, 700);
}
