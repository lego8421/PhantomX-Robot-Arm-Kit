#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <QMainWindow>

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

    typedef struct {
        uint16_t valueMax;
        double angleMax;
        Dynamixel::type type;
    }Info;

    Dynamixel(Dynamixel::type type);

    QByteArray generateJointAnglePacket(dVector q);


    double convertDynamixelToAngle(uint16_t value);
    uint16_t convertAngleToDynamixel(double angle);
    uint16_t convertDualDynamixel(uint16_t value);

private:
    const double LINK_OFFSET = 17.0;
    const double ANGLE_OFFSET = 90.0;

    Info _info;
};

#endif // DYNAMIXEL_H
