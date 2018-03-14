#include "dynamixel.h"

#include <QDebug>

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

    QByteArray buffer;
    uint16_t parameter[5] = {0,};
    int parameterLength = 15;

    buffer[0] = 0xFF;
    buffer[1] = 0xFF;

    buffer[2] = 0xFE;
    buffer[3] = parameterLength + 2;
    buffer[4] = 0x83;

    for(int i=0;i<5;i++) {
        parameter[i] = convertAngleToDynamixel(q[i] * _RAD2DEG);
    }

    // yaw
    buffer[5] = 1;
    buffer[6] = parameter[0] & 0x00FF;
    buffer[7] = parameter[0] >> 8;

    // pitch 1
    buffer[8] = 2;
    buffer[9] = parameter[1] & 0x00FF;
    buffer[10] = parameter[1] >> 8;

    buffer[11] = 3;
    buffer[12] = (_info.valueMax - parameter[1]) & 0x00FF;
    buffer[13] = (_info.valueMax - parameter[1]) >> 8;


    qDebug() << buffer;

    return buffer;


//    unsigned char CheckSum = 0;
//    unsigned char PacketLength = ParameterLength + 6;

//    for (Count = 2; Count < PacketLength - 1; Count++)  // Except 0xFF, Checksum
//    {
//        CheckSum += TxBuffer[Count];
//    }

//    TxBuffer[Count] = ~CheckSum;
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
