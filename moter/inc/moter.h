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

#define PI 3.1415926f



void Send_Data_Dj(CAN_HandleTypeDef *hcan,uint16_t ID,int16_t Control_value1=0,int16_t Control_value2=0,int16_t Control_value3=0,int16_t Control_value4=0);
void Send_Data_DM(CAN_HandleTypeDef *hcan,uint16_t CAN_ID , float Position_Value , float Velocity_Value);


float Mode_Long(float a,float b);



















#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif



#endif

