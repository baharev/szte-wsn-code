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
#include "RecordSelector.hpp"
#include "CustomSqlQueryModel.hpp"
#include "Person.hpp"

namespace {

const char SELECT[] = "SELECT record.id, person.id, person.name, person.birthday, motion.type, record.date_added "

                      "FROM record JOIN person ON (record.person=person.id) JOIN motion ON (record.type=motion.id) ";

const char ORDER_BY[] = "ORDER BY person.name, person.birthday, record.date_added DESC";

enum Columns {
    REC_ID,
    PERSON_ID,
    NAME,
    BIRTH,
    TYPE,
    ADDED,
    NUMBER_OF_COLUMNS
};

}

RecordSelector::RecordSelector() :
        model(0),
        view(new QTableView),
        nameInput(new QLineEdit),
        clearBtn(createButton("Clear")),
        delBtn(createButton("Delete selected"))
{
    QVBoxLayout* layout = new QVBoxLayout;

    layout->addLayout( createInputLine() );

    layout->addLayout( createControlButtons() );

    layout->addWidget(new QLabel("Double-click on a record to use:"));

    setupModel();

    setupView();

    layout->addWidget(view);

    setLayout(layout);

    setWindowModality(Qt::ApplicationModal);
}

RecordSelector::~RecordSelector() {

    // FIXME Resources are leaked -- not clear how to reclaim them
}

void RecordSelector::checkForError(const QSqlError& error) {

    if (error.isValid()) {

        displayError(error.databaseText()+'\n'+error.driverText());
    }
}

QHBoxLayout* RecordSelector::createInputLine() {

    QHBoxLayout* line = new QHBoxLayout;

    line->addWidget(new QLabel("Name: "));

    connect(nameInput, SIGNAL(textChanged(QString)), SLOT(nameEdited(QString)));

    line->addWidget(nameInput);

    return line;
}

QHBoxLayout* RecordSelector::createControlButtons() {

    QHBoxLayout* buttons = new QHBoxLayout;

    connect(clearBtn, SIGNAL(clicked()), SLOT(clearClicked()));

    connect(delBtn, SIGNAL(clicked()), SLOT(deleteClicked()));

    buttons->addWidget(clearBtn);

    buttons->addWidget(delBtn);

    buttons->addStretch();

    return buttons;
}

QPushButton* RecordSelector::createButton(const char text[]) const {

    QPushButton* button = new QPushButton(text);

    button->setFixedWidth(pixelWidth(text) + 20);

    return button;
}

void RecordSelector::setupModel() {

    CustomSqlQueryModel<Columns, NUMBER_OF_COLUMNS>* customModel = new CustomSqlQueryModel<Columns, NUMBER_OF_COLUMNS>;

    customModel->setAlignment(BIRTH, Qt::AlignRight | Qt::AlignVCenter);

    customModel->setAlignment(ADDED, Qt::AlignHCenter | Qt::AlignVCenter);

    model = customModel;

    model->setHeaderData(REC_ID, Qt::Horizontal, "Rec ID");

    model->setHeaderData(PERSON_ID, Qt::Horizontal, "Person ID");

    model->setHeaderData(NAME, Qt::Horizontal, "Name");

    model->setHeaderData(BIRTH, Qt::Horizontal, "Date of birth");

    model->setHeaderData(TYPE, Qt::Horizontal, "Type of motion");

    model->setHeaderData(ADDED, Qt::Horizontal, "Recorded on");

    setSelectQuery("");
}

void RecordSelector::setupView() {

    view->setModel(model);

    view->verticalHeader()->hide();

    //view->hideColumn(REC_ID);

    view->horizontalHeader()->setStretchLastSection(true);

    view->resizeColumnsToContents();

    int pixelsWide = pixelWidth("     2000-00-00");

    view->setColumnWidth(BIRTH, pixelsWide);

    view->setSelectionBehavior(QAbstractItemView::SelectRows);

    view->setSelectionMode(QAbstractItemView::SingleSelection);

    view->setAlternatingRowColors(true);

    connect(view, SIGNAL(activated(QModelIndex)), SLOT(itemActivated(QModelIndex)) );
}

void RecordSelector::setSelectQuery(const QString& whereClause) {

    const QString query = QString(SELECT)+whereClause+QString(ORDER_BY);

    qDebug() << "executing: " << query;

    model->setQuery(query);

    checkForError(model->lastError());
}

void RecordSelector::setSelectQueryLikeName() {

    const QString name = this->name();

    if (name.length()==0) {

        setSelectQuery("");
    }
    else {

        setSelectQuery("WHERE name LIKE '"+name+"%' ");
    }
}

qint64 RecordSelector::executeRawSQL(const QString& rawSQL) {

    qDebug() << "Raw SQL: " << rawSQL;

    QSqlQuery sql(rawSQL);

    //qDebug() << sql.driver()->hasFeature(QSqlDriver::LastInsertId);

    checkForError(sql.lastError());

    QVariant newID = sql.lastInsertId();

    qint64 id = newID.isValid() ? toInt64(newID) : -1;

    setSelectQueryLikeName();

    return id;
}

void RecordSelector::nameEdited(const QString& ) {

    setSelectQueryLikeName();
}

void RecordSelector::itemActivated(const QModelIndex& item) {

    const int row = item.row();

    qint64 id = getPersonID(row);

    QString name = getName(row);

    QDate birth = getDate(row);

    //emit recordSelected(Person(id, name, birth));

    close();
}

void RecordSelector::clearClicked() {

    nameInput->clear();

    nameInput->setFocus();

    view->clearSelection();

    setSelectQueryLikeName();
}

void RecordSelector::deleteClicked() {

    QModelIndexList selected = view->selectionModel()->selectedIndexes();

    if (selected.empty()) {

        displayWarning("Please select a row to delete!");

        return;
    }

    if (displayQuestion("Are you sure you want to delete the selected record?"))
    {
        qint64 id = getRecordID(selected.first().row());

        deleteRecord(id);
    }
}

void RecordSelector::deleteRecord(const qint64 id) {

    executeRawSQL("DELETE FROM record WHERE id = "+QString::number(id));
}

QSize RecordSelector::minimumSizeHint() const {

    return QSize(300, 300);
}

QSize RecordSelector::sizeHint() const {

    return QSize(750, 700);
}

const QDate RecordSelector::getDate(int row) const {

    Q_ASSERT( 0<=row && row < model->rowCount() );

    QModelIndex birthCol = model->index(row, BIRTH);

    return model->data(birthCol).toDate();
}

const QString RecordSelector::getName(int row) const {

    Q_ASSERT( 0<=row && row < model->rowCount() );

    QModelIndex nameCol = model->index(row, NAME);

    return model->data(nameCol).toString();
}

qint64 RecordSelector::getRecordID(int row) {

    Q_ASSERT( 0<=row && row < model->rowCount() );

    QModelIndex idCol = model->index(row, REC_ID);

    QVariant personID = model->data(idCol);

    return toInt64(personID);
}

qint64 RecordSelector::getPersonID(int row) {

    Q_ASSERT( 0<=row && row < model->rowCount() );

    QModelIndex idCol = model->index(row, PERSON_ID);

    QVariant personID = model->data(idCol);

    return toInt64(personID);
}

qint64 RecordSelector::toInt64(const QVariant& var) {

    bool success = false;

    qint64 int64Value =var.toLongLong(&success);

    if (!success) {

        displayError("Failed to convert the ID to int64");
    }

    return int64Value;
}

int RecordSelector::pixelWidth(const char text[]) const {

    QFont defaultFont;

    QFontMetrics fm(defaultFont);

    return fm.width(text);
}

const QString RecordSelector::name() const {

    return nameInput->text().toUpper().trimmed();
}

void RecordSelector::displayError(const QString& msg) {

    QMessageBox::critical(this, "Fatal error", msg);
    exit(EXIT_FAILURE);
}

void RecordSelector::displayWarning(const QString& msg) {

    QMessageBox::warning(this, "Error", msg);
}

bool RecordSelector::displayQuestion(const QString& question) {

    int ret = QMessageBox::question(this, "Warning", question, QMessageBox::Yes, QMessageBox::Cancel);

    return (ret == QMessageBox::Yes)? true : false ;
}
