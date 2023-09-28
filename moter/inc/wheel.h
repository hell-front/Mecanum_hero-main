#ifndef WHEEL_H
#define WHEEL_H


#include "main.h"
#include "main.h"


class wheel_driver
{
private:
    int32_t current_real;
    float location_real;
    int32_t velocity_real;
    int16_t temperature;



public:
    CAN_HandleTypeDef *hcan;
    uint8_t CAN_ID;

    int32_t current_target;
    float location_target;
    int32_t velocity_target;

    int16_t acceleration;
    int16_t deceleration;

    void wheel_init(uint8_t ID);
    
    void set_current_real(int32_t CURRENT){current_real=CURRENT;}
    int32_t get_current_real(){return current_real;}

    void set_velocity_real(int32_t VELOCITY){velocity_real=VELOCITY;}
    int32_t get_velocity_real(){return velocity_real;}

    void set_temperature_real(int16_t TEMPERATURE){temperature=TEMPERATURE;}
    int16_t get_temperature_real(){return temperature;}


    void Driver_Enable();
    void Velocity_Servo_Enable();
    void Location_Servo_Enable();
    void Current_Servo_Enable();
    void send_velocity();
    void take_driver_parameter();

    void Can_Data_processing(uint32_t ID,uint8_t buf[]);

};






#endif



