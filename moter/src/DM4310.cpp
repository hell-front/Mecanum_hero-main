#include "DM4310.h"
#include "main.h"

#ifdef __cplusplus       
extern "C"{                      
#endif

DM4310_moter::DM4310_moter(uint8_t ID,float LOCATION,float LOCATION_ZERO,float LOCATION_MAX,float LOCATION_MIN){
        CAN_ID=ID;
        current_target=0;
        location_target=LOCATION;
        velocity_target=0;
        location_real=0;
        velocity_real=0;
        angle_last=0;
        CAN_update=0;
        temperature=0;
}

void DM4310_moter::Can_Data_processing(uint8_t buf[]){

    float angle_now;

    CAN_update++;
    state = buf[0]>>4
    location_real = float((int16_t)((buf[1]<<8)|(buf[2])));
    torque_real = float((int16_t)((buf[3]<<4)|(buf[4]>>4)));
    ttorque_real = float((buf[4]&0xF)<<8|buf[5]);
    

mtr.pos = uint_to_float(mtr.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
mtr.vel = uint_to_float(mtr.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
mtr.toq = uint_to_float(mtr.t_int, T_MIN, T_MAX, 12);  // (-18.0,18.0)
mtr.Tmos = (float)(_hcan->pRxMsg->Data[6]);
mtr.Tcoil = (float)(_hcan->pRxMsg->Data[7]);



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