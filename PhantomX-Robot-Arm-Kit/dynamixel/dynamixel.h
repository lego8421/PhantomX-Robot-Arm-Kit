#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <QMainWindow>

#include "kinematics/kinematics.h"

class Dynamixel
{
public:

    enum type{
        DX = 0,
        AX = 1,
        RX = 2,
        EX = 3,
        MX = 4
    };

    typedef struct {
        double valueMax;
        double angleMax;
        Dynamixel::type type;
    }Info;

    Dynamixel(Dynamixel::type type);

    QByteArray generateJointAnglePacket(dVector &q);


    double convertDynamixelToAngle(double value);
    double convertAngleToDynamixel(double angle);

private:
    Info _info;
};

#endif // DYNAMIXEL_H
