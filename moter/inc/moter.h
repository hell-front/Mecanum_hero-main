#ifndef MOTER_H
#define MOTER_H

#ifdef __cplusplus       //-----------��׼д��-----------
extern "C"{                      //-----------��׼д��-----------
#endif

#include "M2006.h"
#include "M3508.h"
#include "DM.h"
#include "GM6020.h"
#include "wheel.h"
#include "math.h"




void Send_Data_Dj(CAN_HandleTypeDef *hcan,uint16_t ID,int16_t Control_value1=0,int16_t Control_value2=0,int16_t Control_value3=0,int16_t Control_value4=0);
void Send_Data_DM_PV(CAN_HandleTypeDef *hcan,uint16_t REAL_CAN_ID , float angle , float Velocity_Value);
//其中CAN的ID为配置电机的ID，后两项分别为位置的取值，以及速度的限制
//角度值进行换算,angle对应的是电机实际的角度值，
// “取值的范围是目前是20度到95度”
// location为对应的弧度值
void Send_Data_DM_Mit(CAN_HandleTypeDef *hcan,uint16_t REAL_CAN_ID , float Position_Value,float angle,float KP_Value,float KD_Value,float Torque_Value);//location的取值范围是正负12.5(理论上)，velocity的取值范围是正负45(理论上)

//velocity的取值范围是正负45
//其中CAN的ID为配置电机的ID，后两项分别为位置的取值，以及速度的限制
void Send_Data_DM_V(CAN_HandleTypeDef *hcan,uint16_t REAL_CAN_ID , float Velocity_Value);

//mode为1时，将使能电机，mode为0时，将失能电机，mode为2时，将保存位置零点，mode为3时，将清除错误（例如过热）
//Control_Mode为0时，mit模式;为1时，位置速度模式，2:速度模式
void Send_Data_DM_Control(CAN_HandleTypeDef *hcan,uint16_t REAL_CAN_ID , int Mode , int Control_Mode);


float Mode_Long(float a,float b);



















#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif



#endif

