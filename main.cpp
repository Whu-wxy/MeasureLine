#include "measuretool.h"
#include <QApplication>
#include <QTime>
#include <QDebug>
#include "measuretool.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QTime time;
    time.start();
    MeasureTool tool;
    tool.setFilePath("C:/Users/Administrator/Desktop/1.JPG");
    qDebug()<<"start";
    tool.startMeasure();

    qDebug()<<time.elapsed()/1000.0<<"s";

    return a.exec();
}
