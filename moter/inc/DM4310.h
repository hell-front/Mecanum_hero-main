#ifndef DM4310_H
#define DM4310_H

#ifdef __cplusplus       
extern "C"{                      
#endif

#include "main.h"
#include "PID.h"

#define   POSITION_MAX 12.5
#define   POSITION_MIN -12.5
#define   VELOCITY_MAX 45.0
#define   VELOCITY_MIN -45.0
#define   TORQUE_MAX 18.0
#define   TORQUE_MIN -18.0
#define   KP_MAX 500.0
#define   KP_MIN 0.0
#define   KD_MAX 5.0
#define   KD_MIN 0.0


float   uint_to_float(int x_int, float x_min, float x_max, int bits);
//计算接收到的数据
int     float_to_uint(float x, float x_min, float x_max, int bits);
//计算准备发送的数据

class DM4310_motor
{
    private:
    public:
    
    DM4310_motor(uint8_t ID,float GEAR_RATIO=10);
    uint8_t CAN_ID;

    uint8_t CAN_update;//检测can数据是否更新
    uint8_t state;//电机运行状态，>=8时有错误状态
    float   velocity_target;//限制速度
    float   position_target;//位置速度控制模式下给定实际位置，限制速度

    float   torque_real;
    float   velocity_real;
    float   position_real;
    float   Temperaturemos;
    float   Temperaturecoil;
    float   position_zero
    
    void    Can_Data_processing(uint8_t buf[]);


    Class_PID location_PID;
    Class_PID velocity_PID;
    
};

#ifdef __cplusplus      
}
#endif



#endif