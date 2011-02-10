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
#include <QDebug>
#include <QFontMetrics>
#include <QLabel>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QHeaderView>
#include <QMessageBox>
#include <QPushButton>
#include <QString>
#include <QSqlError>
#include <QSqlQuery>
#include <QSqlQueryModel>
#include <QTableView>
#include "SQLDialog.hpp"

SQLDialog::SQLDialog() :
        model(new QSqlQueryModel),
        view(new QTableView),
        nameInput(new QLineEdit),
        dateInput(new QDateEdit(QDate::currentDate())),
        useBtn(createButton("Use")),
        clearBtn(createButton("Clear")),
        newBtn(createButton("New person"))
{
    connectToDatabase();

    QVBoxLayout* layout = new QVBoxLayout;

    layout->addLayout( createInputLine() );

    layout->addLayout( createControlButtons() );

    createModel();

    createView();

    layout->addWidget(view);

    setLayout(layout);

    setWindowModality(Qt::ApplicationModal);
}

void SQLDialog::setQuerySelectAll() {

    model->setQuery("SELECT id, name, birthday, date_added FROM person ORDER BY name, birthday");
}

void SQLDialog::setQuerySelectLike(const QString& name) {

    if (name.length()==0) {

        setQuerySelectAll();
    }
    else {

        model->setQuery("SELECT id, name, birthday, date_added FROM person WHERE name LIKE '"+name.toUpper()+"%' ORDER BY name, birthday");
    }
}

void SQLDialog::connectToDatabase() {

    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");

    db.setDatabaseName("../records.sqlite");

    if (!db.open()) {
        QMessageBox::critical(this, "Failed to open the DB",
                              "Failed to open the database of the records!");
    }
}

void SQLDialog::createModel() {

    setQuerySelectAll();

    model->setHeaderData(NAME, Qt::Horizontal, "Name");

    model->setHeaderData(BIRTH, Qt::Horizontal, "Date of birth");

    model->setHeaderData(ADDED, Qt::Horizontal, "Added on");
}

QHBoxLayout* SQLDialog::createInputLine() {

    QHBoxLayout* line = new QHBoxLayout;

    line->addWidget(new QLabel("Name: "));

    connect(nameInput, SIGNAL(textChanged(QString)), SLOT(nameEdited(QString)));

    line->addWidget(nameInput);

    line->addWidget(new QLabel("Born: "));

    dateInput->setDisplayFormat("MMM d yyyy");

    line->addWidget(dateInput);

    return line;
}

QHBoxLayout* SQLDialog::createControlButtons() {

    QHBoxLayout* buttons = new QHBoxLayout;

    connect(useBtn, SIGNAL(clicked()), SLOT(useClicked()));

    connect(clearBtn, SIGNAL(clicked()), SLOT(clearClicked()));

    buttons->addWidget(useBtn);

    buttons->addWidget(clearBtn);

    buttons->addWidget(newBtn);

    buttons->addStretch();

    return buttons;
}

QPushButton* SQLDialog::createButton(const char text[]) {

    QPushButton* button = new QPushButton(text);

    button->setFixedWidth(pixelWidth(text) + 20);

    return button;
}

void SQLDialog::createView() {

    view->setModel(model);

    view->verticalHeader()->hide();

    view->hideColumn(ID);

    view->horizontalHeader()->setStretchLastSection(true);

    view->resizeColumnsToContents();

    int pixelsWide = pixelWidth("     2000-00-00");

    view->setColumnWidth(BIRTH, pixelsWide);

    connect(view, SIGNAL(doubleClicked(QModelIndex)), SLOT(itemDoubleClicked(QModelIndex)) );
}

void SQLDialog::nameEdited(const QString& name) {

    setQuerySelectLike(name);

    const bool isAmbiguous = model->rowCount() != 1;

    QDate date = (isAmbiguous) ? QDate::currentDate() : getDate(0);

    bool enable = (isAmbiguous) ? false : true;

    dateInput->setDate(date);

    useBtn->setEnabled(enable);
}

void SQLDialog::itemDoubleClicked(const QModelIndex& item) {

    const int row = item.row();

    QString name = getName(row);

    QDate birth = getDate(row);

    nameInput->setText(name);

    dateInput->setDate(birth);

    view->selectRow(item.row());
}

void SQLDialog::useClicked() {

    close();
}

void SQLDialog::clearClicked() {

    nameInput->clear();
}

void SQLDialog::newPerson() {

}

QSize SQLDialog::minimumSizeHint() const {

    return QSize(300, 300);
}

QSize SQLDialog::sizeHint() const {

    return QSize(750, 700);
}

const QDate SQLDialog::getDate(int row) {

    Q_ASSERT( 0<=row && row < model->rowCount() );

    QModelIndex birthCol = model->index(row, BIRTH);

    return model->data(birthCol).toDate();
}

const QString SQLDialog::getName(int row) {

    Q_ASSERT( 0<=row && row < model->rowCount() );

    QModelIndex nameCol = model->index(row, NAME);

    return model->data(nameCol).toString();
}

int SQLDialog::pixelWidth(const char text[]) {

    QFont defaultFont;

    QFontMetrics fm(defaultFont);

    return fm.width(text);
}
