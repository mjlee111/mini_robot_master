#include "../include/mini_robot_master/mainwindow.h"
#include <QApplication>
#include <thread>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w(argc, argv);
    w.show();

    a.exec();
}
