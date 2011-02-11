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

namespace {

const char DATABASE[] = "QSQLITE";

const char DB_NAME[] = "../records.sqlite"; // FIXME Hard-coded constant

const char SELECT[] = "SELECT id, name, birthday, date_added FROM person ";

const char ORDER_BY[] = "ORDER BY name, birthday";

}

SQLDialog::SQLDialog() :
        today(QDate::currentDate()),
        model(new QSqlQueryModel),
        view(new QTableView),
        nameInput(new QLineEdit),
        dateInput(new QDateEdit(today)),
        clearBtn(createButton("Clear")),
        newBtn(createButton("New person"))
{
    connectToDatabase();

    QVBoxLayout* layout = new QVBoxLayout;

    layout->addLayout( createInputLine() );

    layout->addLayout( createControlButtons() );

    layout->addWidget(new QLabel("Double-click on a person to use:"));

    setupModel();

    setupView();

    layout->addWidget(view);

    setLayout(layout);

    setWindowModality(Qt::ApplicationModal);
}

void SQLDialog::closeEvent(QCloseEvent* event) {

    //QSqlDatabase::removeDatabase(QSqlDatabase::database().connectionName());

    QSqlDatabase::database().close();

    QWidget::closeEvent(event);

    //emit closed();
}

SQLDialog::~SQLDialog() {

}

void SQLDialog::executeQuery(const QString& query) {

    model->setQuery(query);

    QSqlError error(model->lastError());

    if (error.isValid()) {

        displayError(error.databaseText()+'\n'+error.driverText());
    }
}

void SQLDialog::setSelectQuery(const QString& whereClause) {

    executeQuery(QString(SELECT)+whereClause+QString(ORDER_BY));
}

void SQLDialog::setSelectQueryLike(const QString& name) {

    if (name.length()==0) {

        setSelectQuery("");
    }
    else {

        setSelectQuery("WHERE name LIKE '"+name.toUpper()+"%' ");
    }
}

void SQLDialog::connectToDatabase() {

    QSqlDatabase db = QSqlDatabase::addDatabase(DATABASE);

    db.setDatabaseName(DB_NAME);

    if (!db.open()) {

        displayError("Failed to open the database of the records!");
    }
}

void SQLDialog::setupModel() {

    setSelectQuery("");

    model->setHeaderData(NAME, Qt::Horizontal, "Name");

    model->setHeaderData(BIRTH, Qt::Horizontal, "Date of birth");

    model->setHeaderData(ADDED, Qt::Horizontal, "Added on");
}

QHBoxLayout* SQLDialog::createInputLine() {

    QHBoxLayout* line = new QHBoxLayout;

    line->addWidget(new QLabel("Name: "));

    connect(nameInput, SIGNAL(textChanged(QString)), SLOT(nameEdited(QString)));

    line->addWidget(nameInput);

    line->addWidget(new QLabel("Born (YYYY-MM-DD): "));

    dateInput->setDisplayFormat("yyyy-MM-dd");

    dateInput->setMaximumDate(today);

    line->addWidget(dateInput);

    return line;
}

QHBoxLayout* SQLDialog::createControlButtons() {

    QHBoxLayout* buttons = new QHBoxLayout;

    connect(clearBtn, SIGNAL(clicked()), SLOT(clearClicked()));

    connect(newBtn, SIGNAL(clicked()), SLOT(newPerson()));

    buttons->addWidget(newBtn);

    buttons->addWidget(clearBtn);

    buttons->addStretch();

    return buttons;
}

QPushButton* SQLDialog::createButton(const char text[]) const {

    QPushButton* button = new QPushButton(text);

    button->setFixedWidth(pixelWidth(text) + 20);

    return button;
}

void SQLDialog::setupView() {

    view->setModel(model);

    view->verticalHeader()->hide();

    view->hideColumn(ID);

    view->horizontalHeader()->setStretchLastSection(true);

    view->resizeColumnsToContents();

    int pixelsWide = pixelWidth("     2000-00-00");

    view->setColumnWidth(BIRTH, pixelsWide);

    view->setSelectionBehavior(QAbstractItemView::SelectRows);

    connect(view, SIGNAL(activated(QModelIndex)), SLOT(itemActivated(QModelIndex)) );
}

void SQLDialog::nameEdited(const QString& name) {

    setSelectQueryLike(name);

    dateInput->setDate(today);
}

void SQLDialog::itemActivated(const QModelIndex& item) {

    const int row = item.row();

    QString name = getName(row);

    QDate birth = getDate(row);

    close();
}

void SQLDialog::clearClicked() {

    nameInput->clear();

    nameInput->setFocus();
}

void SQLDialog::newPerson() {

    const QString name = nameInput->text();

    if (name.length()==0) {
        displayWarning("Please enter a name!");
        nameInput->setFocus();
        return;
    }

    const QDate birth = dateInput->date();

    if (birth==today) {
        displayWarning("Please check the date of birth!");
        dateInput->setFocus();
        return;
    }

    const QString dateOfBirth = birth.toString(Qt::ISODate);

    setSelectQuery("WHERE name = UPPER('"+ name+"') AND birthday=DATE('"+dateOfBirth+"') ");

    if (model->rowCount()!=0) {
        displayWarning("Please make sure this person is not already in the database!\n\n"
                       "Did you forget to enter the date of birth?");
        nameInput->setFocus();
        return;
    }

    insertNewPerson(name, dateOfBirth);
}


void SQLDialog::insertNewPerson(const QString& name, const QString& birth) {

    executeQuery("INSERT INTO person VALUES (NULL, UPPER('"+name+"'), DATE('"+birth+"'), DATETIME('now') );");

    close();
}

QSize SQLDialog::minimumSizeHint() const {

    return QSize(300, 300);
}

QSize SQLDialog::sizeHint() const {

    return QSize(750, 700);
}

const QDate SQLDialog::getDate(int row) const {

    Q_ASSERT( 0<=row && row < model->rowCount() );

    QModelIndex birthCol = model->index(row, BIRTH);

    return model->data(birthCol).toDate();
}

const QString SQLDialog::getName(int row) const {

    Q_ASSERT( 0<=row && row < model->rowCount() );

    QModelIndex nameCol = model->index(row, NAME);

    return model->data(nameCol).toString();
}

int SQLDialog::pixelWidth(const char text[]) const {

    QFont defaultFont;

    QFontMetrics fm(defaultFont);

    return fm.width(text);
}

void SQLDialog::displayError(const QString& msg) {

    QMessageBox::critical(this, "Fatal error", msg);
    exit(EXIT_FAILURE);
}

void SQLDialog::displayWarning(const QString& msg) {

    QMessageBox::warning(this, "Error", msg);
}
