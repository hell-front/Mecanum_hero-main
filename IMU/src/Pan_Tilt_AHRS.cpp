/**
 ******************************************************************************
 * COPYRIGHT(C) USTC-ROBOWALKER
 * @file    pan_tilt_AHRS.cpp
 * @author  Glorill 1615688664@qq.com
 * @brief
 * @date    2021-12-11
 * @version 1.0
 *
 ******************************************************************************
 */

#include "pan_tilt_AHRS.h"
//#include "PID.h"
uint8_t Pan_Tilt_AHRS_Rx_Buff[Pan_Tilt_AHRS_RX_BUFFER_SIZE];
const float _PI = 3.1415926535;

void C_PAN_TILT_AHRS::DataCapture(uint8_t *data, uint16_t len)
{
    if (CheckSum(data, len) == false)
    {
        return;
    }
    if (data[0] == 0x55 && data[1] == 0x53)
    {
        INS_Angle[2] = ((int16_t)(((int16_t)data[3] << 8) | data[2])) / 32768.0f * _PI;
        INS_Angle[1] = ((int16_t)(((int16_t)data[5] << 8) | data[4])) / 32768.0f * _PI;
        INS_Angle[0] = ((int16_t)(((int16_t)data[7] << 8) | data[6])) / 32768.0f * _PI;
        Temperature = ((int16_t)(((int16_t)data[9] << 8) | data[8])) / 100.0f;
    }
    else if(data[0] == 0x55 && data[1] == 0x52)
    {
        INS_Omega[2] = ((int16_t)(((int16_t)data[3] << 8) | data[2])) / 32768.0f * 2000.0f; //deg per s
        INS_Omega[1] = ((int16_t)(((int16_t)data[5] << 8) | data[4])) / 32768.0f * 2000.0f; //deg per s
        INS_Omega[0] = ((int16_t)(((int16_t)data[7] << 8) | data[6])) / 32768.0f * 2000.0f; //deg per s
        Temperature = ((int16_t)(((int16_t)data[9] << 8) | data[8])) / 100.0f;
    }
}

bool C_PAN_TILT_AHRS::CheckSum(uint8_t *data, uint16_t len)
{
    uint8_t sum = 0;
    for (uint16_t i = 0; i < len - 1; i++)
    {
        sum += data[i];
    }
    if (sum == data[len - 1])
    {
        return true;
    }
    else
    {
        return false;
    }
}

//航向角-弧度-上为正
float C_PAN_TILT_AHRS::GetYaw(void)
{
    return INS_Angle[0];
}

//俯仰角-弧度-右为正
float C_PAN_TILT_AHRS::GetPitch(void)
{
    return INS_Angle[1];
}

//滚转角-弧度-前为正
float C_PAN_TILT_AHRS::GetRoll(void)
{
    return INS_Angle[2];
}


float C_PAN_TILT_AHRS::GetYawOmega(void)
{
    return INS_Omega[0];
}

//俯仰角-弧度-右为正
float C_PAN_TILT_AHRS::GetPitchOmega(void)
{
    return INS_Omega[1];
}

//滚转角-弧度-前为正
float C_PAN_TILT_AHRS::GetRollOmega(void)
{
    return INS_Omega[2];
}

//温度-摄氏度
float C_PAN_TILT_AHRS::GetTemperature(void)
{
    return Temperature;
}

C_PAN_TILT_AHRS Pan_Tilt_AHRS;

// const float *C_PAN_TILT_AHRS::GetAccel(void)
// {
//     return INS_accel;
// }

// const float *C_PAN_TILT_AHRS::GetGyro(void)
// {
//     return INS_gyro;
// }

// const float *C_PAN_TILT_AHRS::GetMag(void)
// {
//     return INS_mag;
// }

// const float *C_PAN_TILT_AHRS::GetEulerAngle(void)
// {
//     return INS_Angle;
// }
