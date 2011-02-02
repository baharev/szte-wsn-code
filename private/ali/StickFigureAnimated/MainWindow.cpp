
#include <QLayout>
#include <QPushButton>
#include "MainWindow.hpp"
#include "window.hpp"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {

    QPushButton* btn = new QPushButton(this);

    btn->setText("Run");

    btn->setFixedSize(80, 30);

    connect(btn, SIGNAL(clicked()), SLOT(run()));

    setCentralWidget(btn);
}

MainWindow::~MainWindow() {

}

void MainWindow::glwidgetClosed() {

    delete sender();
}

void MainWindow::run() {

    ::window* win = new ::window();

    connect(win, SIGNAL(closed()), SLOT(glwidgetClosed()));

    win->resize(1440, 850);
    win->show();
    win->activateWindow();
}

