#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include "Person.hpp"
#include "MotionTypes.hpp"

class SQLDialog;
class RecordHandler;
class Person;

class MainWindow : public QMainWindow {

        Q_OBJECT

public:
        MainWindow(QWidget *parent = 0);

        ~MainWindow();

private slots:

        void selectPerson();

        void selectRecord();

        void addRecord();

        void onPersonSelected(const Person& person);

        void onRecordSelected(qint64 recID, const Person& person);

private:

        SQLDialog* const dial;
        RecordHandler* const recSelect;

        MotionType type;

        qint64 recordID;

        Person person;
};

#endif // MAINWINDOW_HPP
