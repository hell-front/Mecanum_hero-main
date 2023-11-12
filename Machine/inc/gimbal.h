#ifndef GIMBAL_H
#define GIMBAL_H

#ifdef __cplusplus       //-----------��׼д��-----------
extern "C"{                      //-----------��׼д��-----------
#endif


#include "main.h"
#include "moter.h"
#include "remote.h"
#include "PID.h"







#define Gimbal_Tick (3u)
#define Gimbal_Delta_Pitch (0.20f*Gimbal_Tick)
#define Gimbal_Delta_Yaw (2.0f*Gimbal_Tick)








class Class_Gimbal{

private:




public:
    
    Class_Gimbal(float LOCATION_PITCH=0,float LOCATION_YAW=0){velocity_pitch=0;velocity_yaw=0;location_pitch=LOCATION_PITCH;location_yaw=LOCATION_YAW;gimbal_auto=1;Init_OK=0;}


    float velocity_pitch;
    float velocity_yaw;
    float location_pitch;
    float location_yaw;//主要是从遥控器端输入的角度变化值
    float yaw_real;//表示此时云台和底盘之间的实际yaw角度，在理想跟随情况下应该与location_一样

    float yaw_target;//表示加上视觉修正的合成值
    float pitch_target;

    float yaw_planned;//表示规划后的值
    float pitch_planned;

    uint8_t gimbal_auto;//云台姿态角是否加上视觉的补偿
    uint8_t gimbal_auto_last;//云台姿态角是否加上视觉的补偿

    float yaw_comp;//在视觉模式下云台的补偿
    float pitch_comp;


    uint8_t Init_OK;//标记云台初始化是否完成（即云台是否回到了初始位置上，此时云台的转速应当限制的非常小），如果完成，置为1，如果未完成，则放置为0

    float Delta_Yaw_Max;
    float Delta_Pitch_Max;


    uint16_t Frame_Header;
    int16_t Delta_Yaw;
    int16_t Delta_pitch;
    int16_t Predicted_Delta_Yaw;
    int16_t Predicted_Delta_Pitch;
    int16_t Distance;
    int16_t Predicted_Velocity_Yaw;
    int16_t Predicted_Velocity_Pitch;
    uint16_t Manifold_Flag; 





    float yaw_comp_last;//在视觉模式下云台的补偿
    float pitch_comp_last;   


    Class_PID PID_yaw;

    Class_PID PID_gyro;//用来表示底盘小陀螺的时候的角度的增量，用来避免误差
            

    void UART_Data_processing(uint8_t buf[]);//仅仅在自瞄模式下有用


};



void Gimbal_init();
void Gimbal_resloution();
void Gimbal_PID();
void location_plan_g(float location,float *location_plan,float Delta_location);


















#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif 

#endif
