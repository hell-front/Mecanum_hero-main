#ifndef DM_H
#define DM_H

#ifdef __cplusplus       
extern "C"{                      
#endif

#include "main.h"
#include "moter.h"
#include "PID.h"

#define   LOCATION_MAX 12.5
#define   LOCATION_MIN -12.5
#define   VELOCITY_MAX 45.0
#define   VELOCITY_MIN -45.0
#define   TORQUE_MAX 18.0
#define   TORQUE_MIN -18.0
#define   KP_MAX 500.0
#define   KP_MIN 0.0
#define   KD_MAX 5.0
#define   KD_MIN 0.0
#define   PI 3.1415926f

float   uint_to_float(int x_int, float x_min, float x_max, int bits);//计算接收到的数据
int     float_to_uint(float x, float x_min, float x_max, int bits);//计算准备发送的数据

class DM_motor
{
    private:
    public:
    
    DM_motor(uint8_t ID,float GEAR_RATIO=10);
    uint8_t CAN_ID;

    uint8_t CAN_update;//检测can数据是否更新
    uint8_t state;//电机运行状态，>=8时有错误状态
    float   velocity_target;//最高速度（PV模式），目标追踪速度（V模式）
    float   location_target;//位置速度控制模式下给定实际位置，限制速度

    float   torque_real;//实际力矩值
    float   velocity_real;//实际速度值，单位rad/s
    float   location_real;//实际角度，单位°
    float   Temperaturemos;//mos温度
    float   Temperaturecoil;//线圈温度
    float   location_zero;//角度零点，用以定位
    float   location_min;//主要用于有机械限位的情况下，记录电机位置的最小值，为标准坐标系下（逆时针为正）的值
    float   location_max;//主要用于有机械限位的情况下，记录电机位置的最大值，为标准坐标系下（逆时针为正）的值
    
    void Can_Data_processing(uint8_t buf[]);


    Class_PID location_PID;
    Class_PID velocity_PID;
    
};

#ifdef __cplusplus      
}
#endif



#endif