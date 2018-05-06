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

    enum TabIndex{
        FORWARD = 0,
        INVERSE = 1,
        PATH = 2
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

    QLineEdit ***_lineEditPath;

    Serialport *_serialPort;
    QQueue<QByteArray> *_messageQueue;
    QTimer *_taskTimer;

    const int TASK_TIME = 20;   // 20ms

    CPosOriInverse *_kinematics;
    Joint _q;
    dVector _target;

    QVector<QVector<double>> _node;
    uint32_t _pathCount;

    Dynamixel *_dynamixel;

    void printForwardKinematics();
    void printInverseKinematics();
    dVector getForwardSliderValue();
    dVector getInverseSliderValue();

    void setUiObject();

public slots:
    void forwardValueChanged(int index, int val);
    void inverseValueChanged(int index, int val);

private slots:
    void on_buttonReset_clicked();

    void doUserTask();
    void on_pushButtonPathApply_clicked();
};

#endif // MAINWINDOW_H
