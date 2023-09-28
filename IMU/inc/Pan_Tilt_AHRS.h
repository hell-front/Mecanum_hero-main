/**
  ******************************************************************************
  * COPYRIGHT(C) USTC-ROBOWALKER
  * @file    pan_tilt_AHRS.h
  * @author  Glorill 1615688664@qq.com
  * @brief   云台分电板上的姿态传感器，可返回加速度计、陀螺仪、磁力计、欧拉角等
  * @date    2021-12-11
  * @version 1.0
  *
  ==============================================================================
                            How to use this library
  ==============================================================================
    @note
        适配维特智能的WT931姿态传感器，使用前先连接模块官方上位机配置下波特率、回传信息等
        官网资料：
        https://dl.wit-motion.com:2103/index.html#/wit-service/productLiterature/details?productId=aef9a5d41c5d47f28c968fe12fd2cc8c

  ******************************************************************************
  */

#ifndef __PAN_TILT_AHRS_H__
#define __PAN_TILT_AHRS_H__

#include <stdint.h>
#include "stm32f4xx_hal.h"
//#include "Drivers/Components/drv_uart.h"

#define Pan_Tilt_AHRS_RX_BUFFER_SIZE 50
extern uint8_t Pan_Tilt_AHRS_Rx_Buff[Pan_Tilt_AHRS_RX_BUFFER_SIZE];
class C_PAN_TILT_AHRS
{
    private:
        /*欧拉角 单位 弧度
        右手螺旋定义正负
        0 yaw     上为正
        1 pitch   右为正
        2 roll      前为正*/
        float INS_Angle[3];
        float INS_Omega[3];
        float Temperature;
        // float INS_gyro[3];
        // float INS_accel[3];
        // float INS_mag[3];
        // float INS_quat[4]; //四元数

        bool CheckSum(uint8_t *data, uint16_t len);

    public:
        void DataCapture(uint8_t *data, uint16_t len);

        float GetYaw();
        float GetPitch();
        float GetRoll();
        float GetYawOmega();
        float GetPitchOmega();
        float GetRollOmega();
        float GetTemperature();

        // const float *GetAccel();
        // const float *GetGyro();
        // const float *GetMag();
        // const float *GetEulerAngle();
};

extern C_PAN_TILT_AHRS Pan_Tilt_AHRS;

#endif
