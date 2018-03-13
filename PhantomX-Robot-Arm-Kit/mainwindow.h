#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSlider>
#include <QLineEdit>

#include "serialport/serialport.h"
#include "kinematics/kinematics.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    QSlider **_slider;
    QLineEdit **_lineEdit;

    Serialport *_serialPort;

    CKinematics *_kinematics;
    dVector _qDefault;
    dVector _q;

public slots:
    void valueChanged(int index, int val);
private slots:
    void on_buttonReset_clicked();
};

#endif // MAINWINDOW_H
