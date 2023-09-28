#include "GM6020.h"
#include "main.h"




GM6020_moter::GM6020_moter(uint8_t ID,float LOCATION,float LOCATION_ZERO,float LOCATION_MAX,float LOCATION_MIN){
        CAN_ID=ID;
        current_target=0;
        location_zero=LOCATION_ZERO;
        location_target=LOCATION;
        velocity_target=0;
        location_real=0;
        PROCESSED=0;
        voltage=0;
        velocity_real=0;
        angle_last=0;
        CAN_update=0;
        temperature=0;
        location_max=LOCATION_MAX;
        location_min=LOCATION_MIN;
}



void GM6020_moter::Can_Data_processing(uint8_t buf[]){
        
        float angle_now;
        CAN_update++;

        
        angle_now=((int16_t)((buf[0]<<8)|(buf[1])))*360.0f/8191.0f;
        velocity_real=6.0f*((int16_t)((buf[2]<<8)|(buf[3])));
        current_real=((int16_t)((buf[4]<<8)|(buf[5])));
        temperature=(int8_t)buf[6];


        if(PROCESSED==0){
                set_location_real(angle_now);
                PROCESSED=1;
        }else{
                if (angle_now-angle_last<=-360.0f*0.5f){
                        set_location_real(get_location_real()+angle_now-angle_last+360.0f);
                }
                if(angle_now-angle_last>=360.0*0.5){
                        set_location_real(get_location_real()+angle_now-angle_last-360.0f);
                }
                if ((angle_now-angle_last)>-360.0f*0.5f&&(angle_now-angle_last)<360.0f*0.5f){
                        set_location_real(get_location_real()+angle_now-angle_last);
                }
        }

        


        angle_last=angle_now;
}

