#include "serialport.h"

Serialport::Serialport(QWidget *parent) :
    QSerialPort(parent)
{

    _serialPortScanTimer = new QTimer(parent);
    connect(_serialPortScanTimer,SIGNAL(timeout()),this,SLOT(serialPortScanTimer()));

    _serialPortDisconnectedTimer = new QTimer(parent);
    connect(_serialPortDisconnectedTimer,SIGNAL(timeout()),this,SLOT(serialPortDisconnectedTimer()));
}

void Serialport::startSerialPortScan() {
    _serialPortScanTimer->start(TIMER_INTERVAL);
}

void Serialport::stopSerialPortScan() {
    _serialPortScanTimer->stop();
}

void Serialport::serialPortScanTimer() {

    QList<QSerialPortInfo> serialProtInfoList = QSerialPortInfo::availablePorts();

    for(int i=0;i<serialProtInfoList.size();i++) {
        if(serialProtInfoList.at(i).vendorIdentifier() == 1027 && serialProtInfoList.at(i).productIdentifier() == 24596) {
            if(!serialProtInfoList.at(i).isBusy()) {
                setPortName(serialProtInfoList.at(i).portName());
                setBaudRate(1000000);
                setDataBits(QSerialPort::Data8);
                setParity(QSerialPort::NoParity);
                setFlowControl(QSerialPort::NoFlowControl);

                if(open(QIODevice::ReadWrite)) {
                    _serialPortDisconnectedTimer->start(TIMER_INTERVAL);
                    emit connected(portName());
                }
            }
        }
    }
}

void Serialport::serialPortDisconnectedTimer() {

    QSerialPortInfo *serialPortInfo = new QSerialPortInfo(portName());
    if(!serialPortInfo->isValid()) {
        close();
        _serialPortDisconnectedTimer->stop();
        emit disconnected(portName());
    }
}
