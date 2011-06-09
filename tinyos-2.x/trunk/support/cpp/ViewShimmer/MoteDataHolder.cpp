#include "MoteDataHolder.h"
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QStringList>
#include <QStringListIterator>
#include <QtDebug>
#include "Application.h"

MoteDataHolder::MoteDataHolder(Application &app) : application(app)
{
    progressCounter = 0;
}

MoteDataHolder::~MoteDataHolder()
{
    for(int i = 0; i < motes.size(); i++) delete motes[i];
}

void MoteDataHolder::loadCSVData(QString filename)
{
    QFile f( filename );
    QString line;

    if( f.open( QIODevice::ReadOnly | QIODevice::Text ) ) //file opened successfully
    {
        QTextStream ts( &f );
        line = ts.readLine();

        if(line != "#mote,reboot_ID,length,boot_unix_time,skew_1,offset"){
            QMessageBox msgBox;
            msgBox.setText("Wrong file format! Wrong header!");
            msgBox.exec();
        } else {
            line = ts.readLine();

            //progressBar = new QProgressBar(this);

            while ( !line.isEmpty() && line != "#mote,reboot_ID,unix_time,mote_time,counter,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,volt,temp" )
            {
                createMoteData(line);            //convert line string to mote header data
                line = ts.readLine();         // line of text excluding '\n'
            }

            printMotesHeader();

            //progressBar->show();


            //skip empty lines
            while( line != "#mote,reboot_ID,unix_time,mote_time,counter,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,volt,temp" )
            {
                line = ts.readLine();
            }

            line = ts.readLine();

            while ( !line.isEmpty() && line != "#valami ?" )
            {
                createSample(line);            //convert line string to sample
                line = ts.readLine();         // line of text excluding '\n'
            }

            f.close();
        }
    }

    //progressBar->hide();
    //printMoteData(4);
    qDebug()  << "load finished";
    emit loadFinished();
}

void MoteDataHolder::createMoteData(const QString& line)
{
    MoteData* moteData = new MoteData();


    QStringList list = line.split(",");
    QStringListIterator csvIterator(list);

    if(csvIterator.hasNext()){
        moteData->setParam(MOTEID, csvIterator.next().toInt());
        moteData->setParam(REBOOTID, csvIterator.next().toInt());
        moteData->setParam(LENGTH, csvIterator.next().toDouble());        
        moteData->setParam(BOOT_UNIX_TIME, csvIterator.next().toDouble());
        moteData->setParam(SKEW_1, csvIterator.next().toDouble());
        moteData->setParam(OFFSET, csvIterator.next().toDouble());
    }



    //int prevMax = progressBar->maximum();
    //int length = moteData->getLength()*204.8;
    //progressBar->setMaximum(prevMax+length);

    motes.append(moteData);
}
//#mote,reboot_ID,unix_time,mote_time,counter,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,volt,temp

void MoteDataHolder::createSample(const QString& str)
{
    QStringList list = str.split(",");
    QStringListIterator csvIterator(list);
    Sample sample;
    MoteData* mote;
    int moteID;
    int counter;

    if(csvIterator.hasNext()){
        moteID = csvIterator.next().toInt();
        mote = findMoteID(moteID);

        int reboot_ID = csvIterator.next().toInt();

        sample.unix_time = csvIterator.next().toDouble();

        int mote_time = csvIterator.next().toLong();
        counter = csvIterator.next().toInt();

        sample.xAccel = csvIterator.next().toInt();
        sample.yAccel = csvIterator.next().toInt();
        sample.zAccel = csvIterator.next().toInt();
        sample.xGyro = csvIterator.next().toInt();
        sample.yGyro = csvIterator.next().toInt();
        sample.zGyro = csvIterator.next().toInt();
        sample.voltage = csvIterator.next().toInt();
        sample.temp = csvIterator.next().toInt();
    }

    mote->appendSample(sample);

//    double time = sample.unix_time;
//    double value = (sample.xAccel-2300)/12;
//    QPointF point(time, value);
//    MoteData::instance().append(point);

    //progressBar->setValue(progressCounter++);

}

MoteData* MoteDataHolder::findMoteID(int id)
{
    for(int i=0; i<motes.size(); i++){
        if(motes[i]->getMoteID() == id){
            return motes[i];
        }
    }
    return NULL;
}

void MoteDataHolder::printMoteData(int id)
{
    MoteData* mote = findMoteID(id);

    for(int i = 0; i < mote->size(); i++){
        qDebug() << QString::number(mote->getSampleAt(i).xAccel);
    }

}

void MoteDataHolder::printMotesHeader()
{
    QString text;
    text.append("===============================================\n");
    for(int i=0; i<motes.size(); i++){
        text.append("Mote: " + QString::number(motes[i]->getMoteID()) + ", ");
        text.append(QString::number(motes[i]->getRebootID()) + ", ");
        text.append(QString::number(motes[i]->getLength()) + ", ");
        text.append(QString::number(motes[i]->getBootUnixTime()) + "\n");

        qDebug() << motes[i]->getMoteID();
        qDebug() << motes[i]->getRebootID();
    }

    text.append("===============================================\n");

}