#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    if(argc!=3) { qDebug() << "Must enter robot id as parameter e.g, -r 1" << endl; exit(1); }
    int robot_id = QString::fromLocal8Bit(argv[2]).toInt();
    qDebug() << "Running Teleop for Robot" << robot_id;
    QApplication a(argc, argv);
    MainWindow w(robot_id);
    w.show();

    return a.exec();
}
