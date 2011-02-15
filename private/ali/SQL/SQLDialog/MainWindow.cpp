
#include <QDebug>
#include <QLayout>
#include <QPushButton>
#include "MainWindow.hpp"
#include "SQLDialog.hpp"
#include "RecordSelector.hpp"
#include "Person.hpp"

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent), dial(new SQLDialog), recSelect(new RecordSelector) {


    QHBoxLayout* box = new QHBoxLayout;

    QPushButton* selectBtn = new QPushButton;

    selectBtn->setText("Select Person");

    connect(selectBtn, SIGNAL(clicked()), SLOT(selectPerson()));

    box->addWidget(selectBtn);

    QPushButton* record = new QPushButton;

    record->setText("Record");

    connect(record, SIGNAL(clicked()), SLOT(selectRecord()));

    box->addWidget(record);

    QWidget* centralWidget = new QWidget;

    centralWidget->setLayout(box);

    setCentralWidget(centralWidget);

    connect(dial, SIGNAL(personSelected(Person)), SLOT(onPersonSelected(Person)));

    connect(recSelect, SIGNAL(recordSelected(qint64,Person)), SLOT(onRecordSelected(qint64,Person)));
}

MainWindow::~MainWindow() {

}

void MainWindow::onPersonSelected(const Person& person) {

    qDebug() << "Person selected: " << person.id() << ", " << person.name() << ", " << person.birth().toString(Qt::ISODate);
}

void MainWindow::onRecordSelected(qint64 recID, const Person& person) {

    qDebug() << "Record selected: " << recID << "; (" << person.id() << ", " << person.name() << ", " << person.birth().toString(Qt::ISODate) << ")";
}

void MainWindow::selectPerson() {

    //dial->resize(1440, 850);
    //dial->showMaximized();
    dial->show();
    dial->activateWindow();
}

void MainWindow::selectRecord() {

    recSelect->resize(950, 850);
    //recSelect->showMaximized();
    recSelect->show();
    recSelect->activateWindow();
}
