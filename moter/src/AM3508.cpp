#include "AM3508.h"
#include "main.h"
#include "string.h"



MK20_driver::MK20_driver(uint8_t ID,float GEAR_RATIO){
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
        rotation_angle=0;



}

void MK20_driver::set_zero_position(){
    location_target=0;
    set_location_real(0);
}

//结束电机返回帧为0x300+ID的处理函数，此返回的为绝对位置，如果使用速度模式。则可以用过上位机把此帧给屏蔽掉
void MK20_driver::Can_Data0x300_processing(uint8_t buf[])
{
    rotation_angle=int32_t((buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3]);
    location_out_real=((int16_t)((buf[4]<<8)|(buf[5])))*360.0f/8191.0f;;
    
}

//接受电机返回帧为0x200+ID的处理函数，该帧下和C620的返回格式完全一样，如果要用此返回帧的位置，则必须保证返回频率为1kHZ，否则就有位置丢失的可能
void MK20_driver::Can_Data0x200_processing(uint8_t buf[])
{

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


