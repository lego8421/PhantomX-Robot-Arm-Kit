#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :QMainWindow(parent), ui(new Ui::MainWindow) {

    ui->setupUi(this);

    _slider = new QSlider*[6];
    _slider[0] = ui->horizontalSlider0;
    _slider[1] = ui->horizontalSlider1;
    _slider[2] = ui->horizontalSlider2;
    _slider[3] = ui->horizontalSlider3;
    _slider[4] = ui->horizontalSlider4;
    _slider[5] = ui->horizontalSlider5;

    _lineEdit = new QLineEdit*[6];
    _lineEdit[0] = ui->lineEdit0;
    _lineEdit[1] = ui->lineEdit1;
    _lineEdit[2] = ui->lineEdit2;
    _lineEdit[3] = ui->lineEdit3;
    _lineEdit[4] = ui->lineEdit4;
    _lineEdit[5] = ui->lineEdit5;

    for(int i=0; i<6; i++) {

        // set range, only number
        _lineEdit[i]->setValidator( new QIntValidator(-150, 150, this) );

        // set text signal
        connect(_lineEdit[i], &QLineEdit::textChanged, [=](QString text) { bool ok; _slider[i]->setValue(text.toInt(&ok));});

        // set slider signal
        connect(_slider[i], &QSlider::valueChanged, [=](int value) { valueChanged(i,value); });
    }

    // set link, joint
    _kinematics = new CPosOriInverse(POSITION_ORIENTATION);
    _kinematics->AttachJoint(REVOLUTE_JOINT, 2, 0.0, 0.0, 0.000, -90 * _DEG2RAD, 0 * _DEG2RAD, 0 * _DEG2RAD, 0.01, -180.0 * _DEG2RAD, 180.0 * _DEG2RAD, 0 * _DEG2RAD, 1.);
    _kinematics->AttachJoint(REVOLUTE_JOINT, 0, 0.0, 0.0, 0.150, 0 * _DEG2RAD, 0 * _DEG2RAD, 0 * _DEG2RAD, 0.01, -180.0 * _DEG2RAD, 180.0 * _DEG2RAD, 0 * _DEG2RAD, 1.);
    _kinematics->AttachJoint(REVOLUTE_JOINT, 0, 0.0, 0.0, 0.145, 0 * _DEG2RAD, 0 * _DEG2RAD, 0 * _DEG2RAD, 0.01, -180.0 * _DEG2RAD, 180.0 * _DEG2RAD, 0 * _DEG2RAD, 1.);
    _kinematics->AttachJoint(REVOLUTE_JOINT, 0, 0.0, 0.0, 0.070, 0 * _DEG2RAD, 0 * _DEG2RAD, 0 * _DEG2RAD, 0.01, -180.0 * _DEG2RAD, 180.0 * _DEG2RAD, 0 * _DEG2RAD, 1.);
    _kinematics->AttachJoint(REVOLUTE_JOINT, 2, 0.0, 0.0, 0.080, 0 * _DEG2RAD, 0 * _DEG2RAD, 0 * _DEG2RAD, 0.01, -180.0 * _DEG2RAD, 180.0 * _DEG2RAD, 0 * _DEG2RAD, 1.);
    _kinematics->AttachJoint(PRISMATIC_JOINT, 0, 0.0, 0.0, 0.0, 0 * _DEG2RAD, 0 * _DEG2RAD, 0 * _DEG2RAD, 0.01, -180.0 * _DEG2RAD, 180.0 * _DEG2RAD, 0 * _DEG2RAD, 1.);
    ui->widget->setKinematics(_kinematics);

    // set init position
    _qDefault = dVector(6);
    _qDefault[0] = 0.0;
    _qDefault[1] = 90.0;
    _qDefault[2] = -90.0;
    _qDefault[3] = 0.0;
    _qDefault[4] = 0.0;
    _qDefault[5] = 0.0;

    _q = _qDefault * _DEG2RAD;

    ui->widget->setJointAngle(_q);

    _serialPort = new Serialport(this);
    _serialPort->startSerialPortScan();

    connect(_serialPort, &Serialport::connected, [=](QString portName) { qDebug() << "connected" << portName;});
    connect(_serialPort, &Serialport::disconnected, [=](QString portName) { qDebug() << "disconnected" << portName;});

}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::valueChanged(int index, int val) {

    _q[index] = val * _DEG2RAD;
    ui->widget->setJointAngle(_q);
    ui->widget->updateGL();

    _lineEdit[index]->setText(QString::number(val));
}

void MainWindow::on_buttonReset_clicked() {

    _q = _qDefault * _DEG2RAD;

    ui->widget->setJointAngle(_q);
    ui->widget->updateGL();

    for(int i=0; i<6; i++) {
        _lineEdit[i]->setText(QString::number(_qDefault[i]));
        _slider[i]->setValue(_qDefault[i]);
    }
}
