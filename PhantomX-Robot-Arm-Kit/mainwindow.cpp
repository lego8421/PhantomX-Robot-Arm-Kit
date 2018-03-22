#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :QMainWindow(parent), ui(new Ui::MainWindow) {

    ui->setupUi(this);

    // set ui object
    _sliderForward = new QSlider*[5];
    _sliderForward[0] = ui->horizontalSlider0;
    _sliderForward[1] = ui->horizontalSlider1;
    _sliderForward[2] = ui->horizontalSlider2;
    _sliderForward[3] = ui->horizontalSlider3;
    _sliderForward[4] = ui->horizontalSlider4;

    _lineEditForward = new QLineEdit*[5];
    _lineEditForward[0] = ui->lineEdit0;
    _lineEditForward[1] = ui->lineEdit1;
    _lineEditForward[2] = ui->lineEdit2;
    _lineEditForward[3] = ui->lineEdit3;
    _lineEditForward[4] = ui->lineEdit4;

    _labelForward = new QLabel*[6];
    _labelForward[0] = ui->labelForwardX;
    _labelForward[1] = ui->labelForwardY;
    _labelForward[2] = ui->labelForwardZ;
    _labelForward[3] = ui->labelForwardPhi;
    _labelForward[4] = ui->labelForwardTheta;
    _labelForward[5] = ui->labelForwardPsi;

    for(int i=0; i<5; i++) {

        // set range, only number
        _lineEditForward[i]->setValidator( new QIntValidator(-360, 360, this) );

        // set text signal
        connect(_lineEditForward[i], &QLineEdit::textChanged, [=](QString text) {
            bool ok = false;
            _sliderForward[i]->setValue(text.toInt(&ok));
        });

        // set slider signal
        connect(_sliderForward[i], &QSlider::valueChanged, [=](int value) { forwardValueChanged(i,value); });
    }

    // set ui object
    _sliderInverse = new QSlider*[4];
    _sliderInverse[0] = ui->horizontalSliderX;
    _sliderInverse[1] = ui->horizontalSliderY;
    _sliderInverse[2] = ui->horizontalSliderZ;
    _sliderInverse[3] = ui->horizontalSliderPhi;

    _lineEditInverse = new QLineEdit*[4];
    _lineEditInverse[0] = ui->lineEditX;
    _lineEditInverse[1] = ui->lineEditY;
    _lineEditInverse[2] = ui->lineEditZ;
    _lineEditInverse[3] = ui->lineEditPhi;

    for(int i=0; i<4; i++) {

        // set range, only number
        _lineEditInverse[i]->setValidator( new QIntValidator(-300, 300, this) );

        // set text signal
        connect(_lineEditInverse[i], &QLineEdit::textChanged, [=](QString text) {
            bool ok = false;
            _sliderInverse[i]->setValue(text.toInt(&ok));
        });

        // set slider signal
        connect(_sliderInverse[i], &QSlider::valueChanged, [=](int value) { inverseValueChanged(i,value); });
    }





    // set link, joint
    _kinematics = new CPosOriInverse(POSITION_ORIENTATION);
    _kinematics->AttachJoint(REVOLUTE_JOINT, 2, 0.0, 0.0, 0.000, -90 * _DEG2RAD, 0 * _DEG2RAD, 0 * _DEG2RAD, 0.01, -150.0 * _DEG2RAD, 150.0 * _DEG2RAD, 0 * _DEG2RAD, 1.);
    _kinematics->AttachJoint(REVOLUTE_JOINT, 0, 0.0, 0.0, 0.150, 0 * _DEG2RAD, 0 * _DEG2RAD, 0 * _DEG2RAD, 0.01, (-150.0 + Dynamixel::offset::ANGLE) * _DEG2RAD, (150.0 + Dynamixel::offset::ANGLE) * _DEG2RAD, 0 * _DEG2RAD, 1.);
    _kinematics->AttachJoint(REVOLUTE_JOINT, 0, 0.0, 0.0, 0.145, 0 * _DEG2RAD, 0 * _DEG2RAD, 0 * _DEG2RAD, 0.01, (-150.0 - Dynamixel::offset::ANGLE) * _DEG2RAD, (150.0 - Dynamixel::offset::ANGLE) * _DEG2RAD, 0 * _DEG2RAD, 1.);
    _kinematics->AttachJoint(REVOLUTE_JOINT, 0, 0.0, 0.0, 0.070, 0 * _DEG2RAD, 0 * _DEG2RAD, 0 * _DEG2RAD, 0.01, -150.0 * _DEG2RAD, 150.0 * _DEG2RAD, 0 * _DEG2RAD, 1.);
    _kinematics->AttachJoint(REVOLUTE_JOINT, 2, 0.0, 0.0, 0.080, 0 * _DEG2RAD, 0 * _DEG2RAD, 0 * _DEG2RAD, 0.01, -150.0 * _DEG2RAD, 150.0 * _DEG2RAD, 0 * _DEG2RAD, 1.);
    _kinematics->AttachJoint(PRISMATIC_JOINT, 0, 0.0, 0.0, 0.0, 0 * _DEG2RAD, 0 * _DEG2RAD, 0 * _DEG2RAD, 0.01, -150.0 * _DEG2RAD, 150.0 * _DEG2RAD, 0 * _DEG2RAD, 1.);
    ui->widget->setKinematics(_kinematics);

    // set init position
    _q.init = dVector(5);
    _q.init[0] = 0.0;
    _q.init[1] = 90.0;
    _q.init[2] = -90.0;
    _q.init[3] = 0.0;
    _q.init[4] = 0.0;

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

void MainWindow::printForwardKinematics() {

    dVector value = CTransformMatrix(_kinematics->Forward()).GetPositionOrientation();
    QString text;

    for(int i=0;i<6;i++) {
        if(i < 3) {
            text.sprintf("%4.2f",value[i] * 1000.0);
        } else {
            text.sprintf("%4.2f",value[i] * _RAD2DEG);
        }

        _labelForward[i]->setText(text);
    }
}

void MainWindow::forwardValueChanged(int index, int val) {


    _q.write[index] = val * _DEG2RAD;
    _lineEditForward[index]->setText(QString::number(val));

    if(!_serialPort->isOpen()) {
        _q.receive = _q.write;
        ui->widget->setJointAngle(_q.receive);
        ui->widget->updateGL();

        printForwardKinematics();
    }
}

void MainWindow::inverseValueChanged(int index, int val) {
    _lineEditInverse[index]->setText(QString::number(val));
}

void MainWindow::on_buttonReset_clicked() {

    _q.write = _q.init * _DEG2RAD;

    if(!_serialPort->isOpen()) {
        _q.receive = _q.write;
        ui->widget->setJointAngle(_q.receive);
        ui->widget->updateGL();

        for(int i=0; i<5; i++) {
            _lineEditForward[i]->setText(QString::number(_q.init[i]));
            _sliderForward[i]->setValue(_q.init[i]);
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

        printForwardKinematics();
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
