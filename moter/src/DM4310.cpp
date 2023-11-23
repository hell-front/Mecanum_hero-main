#include "DM4310.h"
#include "main.h"
#include "string.h"

DM4310_motor::DM4310_motor(uint8_t ID,float LOCATION)
{
        CAN_ID=ID;
        position_real=LOCATION;
        velocity_target=0;
        position_real=0;
        position_zero=0;
        velocity_real=0;
        torque_real=0;
        CAN_update=0;
        Temperaturemos=0;
        Temperaturecoil=0;
}

void DM4310_motor::Can_Data_processing(uint8_t buf[])
{

        CAN_update++;
        state = buf[0]>>4;
        position_real = (buf[1]<<8)|(buf[2]);
        velocity_real = (buf[3]<<4)|(buf[4]>>4);
        torque_real = ((buf[4]&0xF)<<8)|(buf[5]);
        Temperaturemos = buf[6];
        Temperaturecoil = buf[7];

        position_real = uint_to_float(position_real,POSITION_MIN,POSITION_MAX,16);// (-12.5,12.5)
        position_real = position_real*180.0f/PI;// (-716.2,716.2)
        velocity_real = uint_to_float(velocity_real,VELOCITY_MIN,VELOCITY_MAX,12);// (-45.0,45.0)
        torque_real = uint_to_float(torque_real,TORQUE_MIN,TORQUE_MAX,12);// (-18.0,18.0)

}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
        /// converts unsigned int to float, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
/// Converts a float to an unsigned int, given range and number of bits///
        float span = x_max - x_min;
        float offset = x_min;
        return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}