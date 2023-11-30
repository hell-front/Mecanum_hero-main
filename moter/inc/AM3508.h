#ifndef AM3508_H
#define AM3508_H

#ifdef __cplusplus       //-----------��׼д��-----------
extern "C"{                      //-----------��׼д��-----------
#endif


#include "main.h"
#include "PID.h"

#define Current_MAX_M3508 16384

class MK20_driver
{
private:
    int16_t current_real;
    float   velocity_real;
    float   location_real;
    int8_t  temperature;
    



public:

    MK20_driver(uint8_t ID,float GEAR_RATIO=19);
    uint8_t CAN_ID;

    uint8_t CAN_update;//检测can数据是否更新
    int16_t current_target;
    float   velocity_target;
    float   location_target;
    float   gear_ratio;

    float angle_last;








    void set_zero_position();

    
    void set_current_real(int16_t CURRENT){current_real=CURRENT;}
    int16_t get_current_real(){return current_real;}

    void set_velocity_real(float VELOCITY){velocity_real=VELOCITY;}
    float get_velocity_real(){return velocity_real;}

    void set_temperature_real(int8_t TEMPERATURE){temperature=TEMPERATURE;}
    int8_t get_temperature_real(){return temperature;}

    void set_location_real(float LOCATION){location_real=LOCATION;}
    float get_location_real(){return location_real;}


    void Can_Data0x200_processing(uint8_t buf[]);
    void Can_Data0x300_processing(uint8_t buf[]);


    
    










    Class_PID location_PID;
    Class_PID velocity_PID;
//    Class_PID current_PID;



};











#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif



#endif
