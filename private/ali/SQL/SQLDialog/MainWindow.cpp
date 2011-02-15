
#include <QDebug>
#include <QFile>
#include <QLayout>
#include <QPushButton>
#include "MainWindow.hpp"
#include "SQLDialog.hpp"
#include "RecordSelector.hpp"

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        dial(new SQLDialog),
        recSelect(new RecordHandler),
        type(RIGHT_ELBOW_FLEX),
        recordID(-1)
{


    QHBoxLayout* box = new QHBoxLayout;

    QPushButton* selectBtn = new QPushButton("Select Person");

    connect(selectBtn, SIGNAL(clicked()), SLOT(selectPerson()));

    box->addWidget(selectBtn);

    QPushButton* record = new QPushButton("Record");

    connect(record, SIGNAL(clicked()), SLOT(selectRecord()));

    box->addWidget(record);

    QPushButton* addRec = new QPushButton("Add record");

    connect(addRec, SIGNAL(clicked()), SLOT(addRecord()));

    box->addWidget(addRec);

    QWidget* centralWidget = new QWidget;

    centralWidget->setLayout(box);

    setCentralWidget(centralWidget);

    connect(dial, SIGNAL(personSelected(Person)), SLOT(onPersonSelected(Person)));

    connect(recSelect, SIGNAL(recordSelected(qint64,Person)), SLOT(onRecordSelected(qint64,Person)));
}

MainWindow::~MainWindow() {

}

void MainWindow::onPersonSelected(const Person& p) {

    qDebug() << "Person selected: " << p.id() << ", " << p.name() << ", " << p.birth().toString(Qt::ISODate);

    person = p;
}

void MainWindow::onRecordSelected(qint64 recID, const Person& p) {

    qDebug() << "Record selected: " << recID << "; (" << p.id() << ", " << p.name() << ", " << p.birth().toString(Qt::ISODate) << ")";

    QFile record("../rec/"+QString::number(recordID)+".csv");

    qDebug() << "File exists: " << record.exists();

    recordID = recID;
    person = p;

    Q_ASSERT(recordID > 0);
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

void MainWindow::addRecord() {

    type = (type == RIGHT_ELBOW_FLEX)? LEFT_ELBOW_FLEX:RIGHT_ELBOW_FLEX;

    recordID = recSelect->insertRecord(person.id(), type);

    qDebug() << "New record inserted with id: " << recordID;
}
