#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <vector>
#include <valarray>

#include <QMainWindow>
#include <QSlider>
#include <QLineEdit>
#include <QLabel>
#include <QQueue>

#include <QCamera>
#include <QCameraInfo>
#include <QCameraViewfinder>
#include <QCameraImageCapture>
#include <QCameraImageProcessing>
#include <QScreen>

#include "kinematics/posOriInverse.h"
#include "interpolation.h"
#include "dynamixel/dynamixel.h"
#include "serialport/serialport.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

    enum TabIndex{
        FORWARD = 0,
        INVERSE = 1,
        PATH = 2,
        CAMERA = 3
    };

    typedef struct {
        dVector init;
        dVector target;
        dVector current;
    }Joint;

    typedef struct {
        std::vector<std::valarray<double>> node;
        std::vector<std::valarray<double>> path;
        std::vector<std::valarray<double>> init;
        uint32_t pathCount;
    }Interpolation;

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

    QCamera             *mCamera;
    QCameraViewfinder   *mCameraViewFinder;
    QCameraInfo         mCameraInfo;
    QPixmap             *mCameraData;

    Serialport *_serialPort;
    QQueue<QByteArray> *_messageQueue;
    QTimer *_taskTimer;

    const int TASK_TIME = 20;   // 20ms

    CPosOriInverse *_kinematics;
    Joint _q;
    dVector _target;

    Interpolation _interpolation;

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
    void on_buttonPathApply_clicked();
};

#endif // MAINWINDOW_H
