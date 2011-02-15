#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>

class SQLDialog;
class RecordSelector;
class Person;

class MainWindow : public QMainWindow {

        Q_OBJECT

public:
        MainWindow(QWidget *parent = 0);

        ~MainWindow();

private slots:

        void selectPerson();

        void selectRecord();

        void onPersonSelected(const Person& person);

private:

        SQLDialog* const dial;
        RecordSelector* const recSelect;
};

#endif // MAINWINDOW_HPP
