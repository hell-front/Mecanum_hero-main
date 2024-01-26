#ifndef TASK_H
#define TASK_H


#include "main.h"
#include "rtthread.h"
#include <rthw.h>




#ifdef __cplusplus       //-----------��׼д��-----------
extern "C"{                      //-----------��׼д��-----------
#endif






void Task_Init(void *parameter);
void LED_Galloping(void *parameter);
void Task_heating_IMU(void *parameter);
void Task_SerialPlot(void *parameter);
void Task_Chassis_tristeer_wheel(void *parameter);
void Task_Gimbal(void *parameter);
void Task_Gimbal_init(void *parameter);
void Task_Shoot(void *parameter);
void Task_Referee(void *parameter);
void Task_test_motor(void *parameter);
void Task_DM_enable(void *parameter);

void MPU6500_Communication(void *parameter);


void Task_Remote_Check(void *parameter);


void Task_Referee_UI(void *parameter);

void Task_Send_data_to_miniPC(void *parameter);


#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif 


#endif
