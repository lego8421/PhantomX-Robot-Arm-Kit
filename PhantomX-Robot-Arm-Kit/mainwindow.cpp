#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :QMainWindow(parent), ui(new Ui::MainWindow) {

    ui->setupUi(this);

    // set ui object
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
        connect(_lineEdit[i], &QLineEdit::textChanged, [=](QString text) {
            bool ok = false;
            _slider[i]->setValue(text.toInt(&ok));
        });

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
    _q.init = dVector(6);
    _q.init[0] = 0.0;
    _q.init[1] = 90.0;
    _q.init[2] = -90.0;
    _q.init[3] = 0.0;
    _q.init[4] = 0.0;
    _q.init[5] = 0.0;

    _q.write = _q.init * _DEG2RAD;
    _q.receive = _q.write;

    ui->widget->setJointAngle(_q.receive);

    _dynamixel = new Dynamixel(Dynamixel::type::AX);

    // set serial port
    _serialPort = new Serialport(this);
    _serialPort->startSerialPortScan();
    _messageQueue = new QQueue<QByteArray>();

    // set serial signal connect
    connect(_serialPort, &Serialport::connected, [=](QString portName) {
        ui->statusBar->showMessage("connected: " + portName,1000);
        _serialPort->write(_dynamixel->generateJointAnglePacket(_q.write));
    });
    connect(_serialPort, &Serialport::disconnected, [=](QString portName) {
        ui->statusBar->showMessage("disconnected: " + portName,1000);
    });
    connect(_serialPort, &Serialport::readyRead, [=]() {
        QByteArray packet;
        _dynamixel->addMessageBuffer(_serialPort->readAll());
        while(_dynamixel->getReceivedPacket(&packet)) {
            _messageQueue->enqueue(packet);
        }
    });

    // set user task timer
    _taskTimer = new QTimer(this);
    _taskTimer->start(20);
    connect(_taskTimer,SIGNAL(timeout()),this,SLOT(doUserTask()));
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::valueChanged(int index, int val) {

    _q.write[index] = val * _DEG2RAD;
    _lineEdit[index]->setText(QString::number(val));

    if(!_serialPort->isOpen()) {
        _q.receive = _q.write;
        ui->widget->setJointAngle(_q.receive);
        ui->widget->updateGL();
    }
}

void MainWindow::on_buttonReset_clicked() {

    _q.write = _q.init * _DEG2RAD;

    if(!_serialPort->isOpen()) {
        _q.receive = _q.write;
        ui->widget->setJointAngle(_q.receive);
        ui->widget->updateGL();

        for(int i=0; i<6; i++) {
            _lineEdit[i]->setText(QString::number(_q.init[i]));
            _slider[i]->setValue(_q.init[i]);
        }
    }
}

void MainWindow::doUserTask() {

    static uint8_t getAngleId = 1;

    if(!_messageQueue->isEmpty()) {
        QByteArray buffer = _messageQueue->dequeue();
        int16_t id = buffer[2];
        uint16_t pos = (((uint16_t)buffer[6] << 8) & 0xFF00) | (buffer[5] & 0xFF);

        switch(id) {
        case 1:
            _q.receive[0] = _dynamixel->convertDynamixelToAngle(pos) * _DEG2RAD;
            break;

        case 3:
            _q.receive[1] = (_dynamixel->convertDynamixelToAngle(pos) - Dynamixel::offset::LINK + Dynamixel::offset::ANGLE) * _DEG2RAD;
            break;

        case 4:
            _q.receive[2] = (_dynamixel->convertDynamixelToAngle(pos) + Dynamixel::offset::LINK - Dynamixel::offset::ANGLE) * _DEG2RAD;
            break;

        case 6:
            _q.receive[3] = _dynamixel->convertDynamixelToAngle(pos) * _DEG2RAD;
            break;

        case 7:
            _q.receive[4] = _dynamixel->convertDynamixelToAngle(pos) * _DEG2RAD;
            break;
        }

        ui->widget->setJointAngle(_q.receive);
        ui->widget->updateGL();
    }

    if(_serialPort->isOpen()) {
        _serialPort->write(_dynamixel->generateJointAnglePacket(_q.write));
        _serialPort->write(_dynamixel->generateGetJointAngleByIdPacket(getAngleId));
        getAngleId++;
        if(getAngleId==8) {
            getAngleId = 1;
        }
    }
}
