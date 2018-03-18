#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <QMainWindow>
#include <iostream>

#include "kinematics/kinematics.h"

class Dynamixel
{
public:

    enum type {
        DX = 0,
        AX = 1,
        RX = 2,
        EX = 3,
        MX = 4
    };

    enum offset {
        ANGLE = 90,
        LINK = 17
    };

    typedef struct {
        uint16_t valueMax;
        double angleMax;
        Dynamixel::type type;
    }Info;

    Dynamixel(Dynamixel::type type);

    QByteArray generateJointAnglePacket(dVector q);
    QByteArray generateGetJointAngleByIdPacket(uint8_t id);
    QByteArray generateTorqueOffPacket();

    void addMessageBuffer(QByteArray buffer);
    bool getReceivedPacket(QByteArray* received);


    double convertDynamixelToAngle(uint16_t value);
    uint16_t convertAngleToDynamixel(double angle);
    uint16_t convertDualDynamixel(uint16_t value);

private:
    Info _info;
    std::string _messageBuffer;

};

#endif // DYNAMIXEL_H
