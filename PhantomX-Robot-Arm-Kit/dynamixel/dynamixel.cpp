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

double Dynamixel::convertDynamixelToAngle(uint16_t value) {

    if(value >= _info.valueMax - 1) {
        value = _info.valueMax;
    }

    return (((double)value / _info.valueMax) * _info.angleMax) - (_info.angleMax/2.0);
}

uint16_t Dynamixel::convertAngleToDynamixel(double angle) {

    uint16_t ret = 0;

    if(angle > _info.angleMax/2.0) {
        angle = _info.angleMax/2.0;
    } else if(angle < -_info.angleMax/2.0) {
        angle = -_info.angleMax/2.0;
    }

    angle += (_info.angleMax/2.0);
    ret = (angle / _info.angleMax) * _info.valueMax;

    if(ret == _info.valueMax) {
        ret = _info.valueMax - 1;
    }

    return ret;
}
