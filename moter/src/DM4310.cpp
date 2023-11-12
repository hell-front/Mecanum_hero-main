#include "DM4310.h"
#include "main.h"
#include "string.h"

DM4310_motor::DM4310_motor(uint8_t ID,float LOCATION)
{
        CAN_ID=ID;
        location_target=LOCATION;
        velocity_target=10;
        location_real=0;
        velocity_real=0;
        torque_real=0;
        CAN_update=0;
        Temperaturemos = 0;
        Temperaturecoil = 0;
}

void DM4310_motor::Can_Data_processing(uint8_t buf[])
{

        CAN_update++;
        state = buf[0]>>4;
        location_real = (float)(((buf[1]<<8)|(buf[2])));
        velocity_real = (float)(((buf[3]<<4)|(buf[4]>>4)));
        torque_real = (float)((buf[4]&0xF)<<8|buf[5]);
        Temperaturemos = (float)(buf[6]);
        Temperaturecoil = (float)(buf[7]);
        
}