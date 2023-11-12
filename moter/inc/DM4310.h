#ifndef DM4310_H
#define DM4310_H

class DM4310_moter
{
    private:
    public:
    
    DM4310_moter(uint8_t ID,float GEAR_RATIO=40);
    uint8_t CAN_ID;

    uint8_t CAN_update;//检测can数据是否更新
    uint8_t state
    float   velocity_target;
    float   location_target;//位置速度控制模式下给定实际位置，限制速度

    float   torque_real;
    float   velocity_real;
    float   location_real;
    float   Temperaturemos;
    float   Temperaturecoil;
    

    float   angle_last;

}

#ifdef __cplusplus      
}
#endif



#endif