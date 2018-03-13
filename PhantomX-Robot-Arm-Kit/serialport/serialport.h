#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>
#include <QDebug>

class Serialport : public QSerialPort
{
    Q_OBJECT
public:
    explicit Serialport(QWidget *parent = 0);

    void startSerialPortScan();
    void stopSerialPortScan();

private:
    const int TIMER_INTERVAL = 100;
    QTimer *_serialPortScanTimer;
    QTimer *_serialPortDisconnectedTimer;

signals:
    void connected(QString portName);
    void disconnected(QString portName);

public slots:
    void serialPortScanTimer();
    void serialPortDisconnectedTimer();
};

#endif // SERIALPORT_H
