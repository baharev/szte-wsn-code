
#include <QLayout>
#include <QPushButton>
#include "MainWindow.hpp"
#include "SQLDialog.hpp"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {

    QPushButton* btn = new QPushButton(this);

    btn->setText("Run");

    btn->setFixedSize(80, 30);

    connect(btn, SIGNAL(clicked()), SLOT(run()));

    setCentralWidget(btn);
}

MainWindow::~MainWindow() {

}

void MainWindow::SQLDialogClosed() {

    //delete sender();
}

void MainWindow::run() {

    SQLDialog* dial = new SQLDialog;

    //connect(dial, SIGNAL(closed()), SLOT(SQLDialogClosed()));

    //dial->resize(1440, 850);
    //dial->showMaximized();
    dial->show();
    dial->activateWindow();
}
