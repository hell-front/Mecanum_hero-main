#ifndef MOTER_H
#define MOTER_H

#ifdef __cplusplus       //-----------��׼д��-----------
extern "C"{                      //-----------��׼д��-----------
#endif

#include "M2006.h"
#include "M3508.h"
#include "DM4310.h"
#include "GM6020.h"
#include "wheel.h"
#include "math.h"

#define PI 3.1416f



void Send_Data_Dj(CAN_HandleTypeDef *hcan,uint16_t ID,int16_t Control_value1=0,int16_t Control_value2=0,int16_t Control_value3=0,int16_t Control_value4=0);
void Send_Data_DM_PV(CAN_HandleTypeDef *hcan,uint16_t REAL_CAN_ID , float Position_Value , float Velocity_Value);
void Send_Data_DM_Control(CAN_HandleTypeDef *hcan,uint16_t REAL_CAN_ID , int Mode , int Control_Mode);
void Send_Data_DM_Mit(CAN_HandleTypeDef *hcan,uint16_t REAL_CAN_ID , float Position_Value,float Velocity_Value,float KP_Value,float KD_Value,float Torque_Value);

float Mode_Long(float a,float b);



















#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif



#endif

