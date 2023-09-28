#include "M3508.h"
#include "main.h"
#include "string.h"



C620_driver::C620_driver(uint8_t ID,float GEAR_RATIO){
        CAN_ID=ID;
        current_target=0;
        location_target=0;
        velocity_target=0;
        gear_ratio=GEAR_RATIO;
        current_real=0;
        location_real=0;
        velocity_real=0;
        temperature=0;
        angle_last=0;
        CAN_update=0;



}

void C620_driver::set_zero_position(){
    location_target=0;
    set_location_real(0);
}




void C620_driver::Can_Data_processing(uint8_t buf[]){

    float angle_now;

    CAN_update++;
    angle_now=((int16_t)((buf[0]<<8)|(buf[1])))*360.0f/8191.0f;
    velocity_real=6.0f*((int16_t)((buf[2]<<8)|(buf[3])))/gear_ratio;
    current_real=((int16_t)((buf[4]<<8)|(buf[5])));
    temperature=(int8_t)buf[6];
    



    if (angle_now-angle_last<=-360.0f*0.5f)
    {
        set_location_real(get_location_real()+(angle_now-angle_last+360.0f)/gear_ratio);//输出轴的转动角度
    }
    if(angle_now-angle_last>=360.0*0.5){
        set_location_real(get_location_real()+(angle_now-angle_last-360.0f)/gear_ratio);
    }
    if ((angle_now-angle_last)>-360.0f*0.5f&&(angle_now-angle_last)<360.0f*0.5f)
    {
        set_location_real(get_location_real()+(angle_now-angle_last)/gear_ratio);
    }
    
    

    angle_last=angle_now;
}


