#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>

class MainWindow : public QMainWindow {

        Q_OBJECT

public:
        MainWindow(QWidget *parent = 0);

        ~MainWindow();

private slots:

        void run();

        void glwidgetClosed();
};

#endif // MAINWINDOW_HPP
