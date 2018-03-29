#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSlider>
#include <QLineEdit>
#include <QLabel>
#include <QQueue>

#include "serialport/serialport.h"
#include "kinematics/posOriInverse.h"
#include "dynamixel/dynamixel.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

    enum KinematicsIndex{
        FORWARD = 0,
        INVERSE = 1
    };

    typedef struct {
        dVector init;
        dVector target;
        dVector current;
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
    QLabel ** _labelInverse;

    Serialport *_serialPort;
    QQueue<QByteArray> *_messageQueue;
    QTimer *_taskTimer;

    CPosOriInverse *_kinematics;
    Joint _q;
    dVector _target;

    Dynamixel *_dynamixel;

    void printForwardKinematics();
    void printInverseKinematics();
    dVector getForwardSliderValue();
    dVector getInverseSliderValue();

public slots:
    void forwardValueChanged(int index, int val);
    void inverseValueChanged(int index, int val);

private slots:
    void on_buttonReset_clicked();

    void doUserTask();
};

#endif // MAINWINDOW_H
