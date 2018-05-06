#include "mainwindow.h"
#include <QApplication>

#define APP_VERSION "1.2.0"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    a.setApplicationVersion(QString(APP_VERSION));
    MainWindow w;
    w.setWindowTitle("PhantomX Robot Arm Kit (v" + QString(APP_VERSION) + ")");
    w.show();

    return a.exec();
}
