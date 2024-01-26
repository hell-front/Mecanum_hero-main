#include "DM.h"
#include "main.h"
#include "string.h"

DM_motor::DM_motor(uint8_t ID,float LOCATION)
{
        CAN_ID=ID;
        location_real=LOCATION;
        velocity_target=0;
        location_real=0;
        location_zero=0;
        velocity_real=0;
        torque_real=0;
        CAN_update=0;
        Temperaturemos=0;
        Temperaturecoil=0;
        location_min=0;
        location_max=360;
}

void DM_motor::Can_Data_processing(uint8_t buf[])
{

        CAN_update++;
        state = buf[0]>>4;
        Temperaturemos = buf[6];
        Temperaturecoil = buf[7];

        location_real = uint_to_float((buf[1]<<8)|(buf[2]),LOCATION_MIN,LOCATION_MAX,16);// (-12.5,12.5)
        location_real = location_real/PI*180.0f;// (-716.2,716.2)
        while((location_real<0.0f)||(location_real>360.0f))
        {
                if(location_real<0.0f)location_real+=360.0f;
                if(location_real>360.0f)location_real-=360.0f;
        }
        velocity_real = uint_to_float((buf[3]<<4)|(buf[4]>>4),VELOCITY_MIN,VELOCITY_MAX,12);// (-45.0,45.0)
        velocity_real = velocity_real/PI*180.0f;// (-2578.3,2578.3)
        
        torque_real = uint_to_float(((buf[4]&0xF)<<8)|(buf[5]),TORQUE_MIN,TORQUE_MAX,12);// (-18.0,18.0)

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