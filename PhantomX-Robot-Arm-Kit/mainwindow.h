#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSlider>
#include <QLineEdit>
#include <QLabel>
#include <QQueue>

#include "serialport/serialport.h"
#include "kinematics/kinematics.h"
#include "dynamixel/dynamixel.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

    typedef struct {
        dVector init;
        dVector write;
        dVector receive;
    }Joint;


public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    QSlider **_sliderForward;
    QLineEdit **_lineEditForward;
    QLabel ** _labelForward;

    QSlider **_sliderInverse;
    QLineEdit **_lineEditInverse;

    Serialport *_serialPort;
    QQueue<QByteArray> *_messageQueue;
    QTimer *_taskTimer;

    CKinematics *_kinematics;
    Joint _q;

    Dynamixel *_dynamixel;

    void printForwardKinematics();

public slots:
    void forwardValueChanged(int index, int val);
    void inverseValueChanged(int index, int val);

private slots:
    void on_buttonReset_clicked();

    void doUserTask();
};

#endif // MAINWINDOW_H
