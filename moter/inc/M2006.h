#ifndef _M2006_H
#define _M2006_H



#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "PID.h"



#ifdef __cplusplus       //-----------��׼д��-----------
extern "C"{                      //-----------��׼д��-----------
#endif


#define Current_MAX_M2006 10000

class C610_driver
{
private:
    int16_t current_real;
    float velocity_real;
    float   location_real;


public:

    
    C610_driver(uint8_t ID,float GEAR_RATIO=36);
    uint8_t CAN_ID;
    uint8_t CAN_update;//检测can数据是否更新
    int16_t current_target;
    float   location_target;
    float velocity_target;
    float   gear_ratio;


    float angle_last;


    


    void set_zero_position();

    void set_current_real(int16_t CURRENT){current_real=CURRENT;}
    int16_t get_current_real(){return current_real;}

    void set_velocity_real(float VELOCITY){velocity_real=VELOCITY;}
    float get_velocity_real(){return velocity_real;}



    void set_location_real(float LOCATION){location_real=LOCATION;}
    float get_location_real(){return location_real;}

    void Can_Data_processing(uint8_t buf[]);





    Class_PID velocity_PID;
    Class_PID location_PID;
//    Class_PID current_PID;




};



void SendM2006_current14(CAN_HandleTypeDef *hcan,int16_t Current1=0,int16_t Current2=0,int16_t Current3=0,int16_t current4=0);
void SendM2006_current58(CAN_HandleTypeDef *hcan,int16_t Current1=0,int16_t Current2=0,int16_t Current3=0,int16_t current4=0);





#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif 



#endif


