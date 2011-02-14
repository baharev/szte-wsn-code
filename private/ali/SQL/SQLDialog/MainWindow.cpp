
#include <QDebug>
#include <QLayout>
#include <QPushButton>
#include "MainWindow.hpp"
#include "SQLDialog.hpp"
#include "Person.hpp"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), dial(new SQLDialog) {

    QPushButton* btn = new QPushButton(this);

    btn->setText("Run");

    btn->setFixedSize(80, 30);

    connect(btn, SIGNAL(clicked()), SLOT(run()));

    setCentralWidget(btn);

    connect(dial, SIGNAL(personSelected(Person)), SLOT(onPersonSelected(Person)));
}

MainWindow::~MainWindow() {

}

void MainWindow::onPersonSelected(const Person& person) {

    qDebug() << "MainWindow received: " << person.id() << ", " << person.name() << ", " << person.birth().toString(Qt::ISODate);
}

void MainWindow::run() {

    //dial->resize(1440, 850);
    //dial->showMaximized();
    dial->show();
    dial->activateWindow();
}
