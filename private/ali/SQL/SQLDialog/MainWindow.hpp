#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>

class SQLDialog;
class Person;

class MainWindow : public QMainWindow {

        Q_OBJECT

public:
        MainWindow(QWidget *parent = 0);

        ~MainWindow();

private slots:

        void run();

        void onPersonSelected(const Person& person);

private:

        SQLDialog* const dial;
};

#endif // MAINWINDOW_HPP
