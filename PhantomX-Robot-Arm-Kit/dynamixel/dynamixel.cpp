#include "dynamixel.h"

Dynamixel::Dynamixel(Dynamixel::type type)
{

    _info.type = type;

    if(type == Dynamixel::type::MX) {
        _info.angleMax = 360;
        _info.valueMax = 2048;
    } else {
        _info.angleMax = 300;
        _info.valueMax = 1024;
    }
}

QByteArray Dynamixel::generateJointAnglePacket(dVector &q) {

}

double Dynamixel::convertDynamixelToAngle(double value) {

    if(value > (_info.valueMax - 1.0)) {
        value = _info.valueMax - 1.0;
    } else if(value < 0.0) {
        value = 0.0;
    }

    return (value / _info.valueMax) * _info.angleMax;
}

double Dynamixel::convertAngleToDynamixel(double angle) {

    if(angle > _info.valueMax/2.0) {
        angle = _info.valueMax/2.0;
    } else if(angle < -_info.valueMax/2.0) {
        angle = -_info.valueMax/2.0;
    }

    return ((angle / _info.angleMax) * _info.valueMax) + (_info.valueMax/2.0);
}
