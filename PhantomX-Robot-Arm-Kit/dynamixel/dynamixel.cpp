#include "dynamixel.h"

#include <QDebug>

Dynamixel::Dynamixel(Dynamixel::type type) {

    _info.type = type;

    if(type == Dynamixel::type::MX) {
        _info.angleMax = 360;
        _info.valueMax = 2048;
    } else {
        _info.angleMax = 300;
        _info.valueMax = 1024;
    }
}

QByteArray Dynamixel::generateJointAnglePacket(dVector q) {

    QByteArray buffer;
    uint16_t parameter[5] = {0,};
    uint8_t parameterLength = 5 * (5+2) + 4;
    uint8_t checkSum = 0;

    q[1] -= ((ANGLE_OFFSET - LINK_OFFSET) * _DEG2RAD);
    q[2] += ((ANGLE_OFFSET - LINK_OFFSET) * _DEG2RAD);

    for(int i=0;i<5;i++) {
        parameter[i] = convertAngleToDynamixel(q[i] * _RAD2DEG);
    }

    // header
    buffer[0] = 0xFF;
    buffer[1] = 0xFF;

    // instruction
    buffer[2] = 0xFE;
    buffer[3] = parameterLength;
    buffer[4] = 0x83;
    buffer[5] = 0x1E;
    buffer[6] = 0x04;


    // yaw
    buffer[7] = 0x01;
    buffer[8] = parameter[0] & 0x00FF;
    buffer[9] = parameter[0] >> 8;
    buffer[10] = 0x00;
    buffer[11] = 0x00;

    // pitch 1
    buffer[12] = 0x02;
    buffer[13] = convertDualDynamixel(parameter[1]) & 0x00FF;
    buffer[14] = convertDualDynamixel(parameter[1]) >> 8;
    buffer[15] = 0x00;
    buffer[16] = 0x00;

    buffer[17] = 0x03;
    buffer[18] = parameter[1] & 0x00FF;
    buffer[19] = parameter[1] >> 8;
    buffer[20] = 0x00;
    buffer[21] = 0x00;

    // pitch 2
    buffer[22] = 0x04;
    buffer[23] = parameter[2] & 0x00FF;
    buffer[24] = parameter[2] >> 8;
    buffer[25] = 0x00;
    buffer[26] = 0x00;

    buffer[27] = 0x05;
    buffer[28] = convertDualDynamixel(parameter[2]) & 0x00FF;
    buffer[29] = convertDualDynamixel(parameter[2]) >> 8;
    buffer[30] = 0x00;
    buffer[31] = 0x00;

    // pitch 3
    buffer[32] = 0x06;
    buffer[33] = parameter[3] & 0x00FF;
    buffer[34] = parameter[3] >> 8;
    buffer[35] = 0x00;
    buffer[36] = 0x00;

    // roll
    buffer[37] = 0x07;
    buffer[38] = parameter[4] & 0x00FF;
    buffer[39] = parameter[4] >> 8;
    buffer[40] = 0x00;
    buffer[41] = 0x00;

    for(int i = 2; i < 42; i++) {
        checkSum += buffer[i];
    }
    buffer[42] = ~checkSum;

    return buffer;
}

QByteArray Dynamixel::generateGetJointAngleByIdPacket(uint8_t id)
{
    QByteArray buffer;
    uint8_t checkSum = 0;

    // header
    buffer[0] = 0xFF;
    buffer[1] = 0xFF;

    // id
    buffer[2] = id;

    // instruction
    buffer[3] = 0x04;
    buffer[4] = 0x02;

    buffer[5] = 0x24;
    buffer[6] = 0x02;

    for(int i = 2; i < 7; i++) {
        checkSum += buffer[i];
    }
    buffer[7] = ~checkSum;

    return buffer;
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

uint16_t Dynamixel::convertDualDynamixel(uint16_t value) {
    return _info.angleMax - value;
}
