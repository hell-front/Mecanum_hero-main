#ifndef GM6020_H
#define GM6020_H

#ifdef __cplusplus       //-----------��׼д��-----------
extern "C"{                      //-----------��׼д��-----------
#endif

#include "PID.h"
#include "main.h"






#define VOLTAGE_MAX_GM6020    30000.0f








class GM6020_moter
{
private:
    int16_t current_real;
    float velocity_real;
    float location_real;
    int8_t temperature;
    



public:
    uint8_t CAN_ID;//电机ID，方便知道布线，无实际用途
    uint8_t CAN_update;//检测can数据是否更新
    int16_t current_target;//目标电流
    float velocity_target;
    float location_target;//电机的目标位置，为标准坐标系下的参数
    int16_t voltage;//电机的激励电压，直接传给电机的参数
    float location_zero;//标记电机零位的值，将电机实际位置与坐标系转换的关键参数
    float location_min;
    float location_max;//记录电机位置的最大值，为标准坐标系下（逆时针为正）的值



    uint8_t PROCESSED;//标志是否为第一次传回角度
    float angle_last;//标记上一次的角度，方便判断是否越过360°

    float Power;//电机的输出功率，用于功率限制使用



    GM6020_moter(uint8_t ID,float LOCATION,float LOCATION_ZERO,float LOCATION_MAX,float LOCATION_MIN);


    void set_current_real(int16_t CURRENT){current_real=CURRENT;}
    int16_t get_current_real(){return current_real;}

    void set_velocity_real(float VELOCITY){velocity_real=VELOCITY;}
    float get_velocity_real(){return velocity_real;}

    void set_temperature_real(int8_t TEMPERATURE){temperature=TEMPERATURE;}
    int8_t get_temperature_real(){return temperature;}

    void set_location_real(float LOCATION){location_real=LOCATION;}
    float get_location_real(){return location_real;}


    void Can_Data_processing(uint8_t buf[]);



    









    Class_PID location_PID;
    Class_PID velocity_PID;
    Class_PID current_PID;



};
















#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif 


#endif
