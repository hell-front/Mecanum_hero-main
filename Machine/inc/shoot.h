#ifndef SHOOT_H
#define SHOOT_H

#ifdef __cplusplus       //-----------��׼д��-----------
extern "C"{                      //-----------��׼д��-----------
#endif

#include "main.h"
#define Friction_SPEED_MAX   6000.0f



#define Shoot_Tick   (3u)
#define Shoot_plate_delta_location   (720*0.001f*Shoot_Tick)

/*爆发优先*/

#define ERUPT_SHOOT_SPEED_FRONT   31750
#define ERUPT_SHOOT_SPEED_BACK  23200//天气一般寒冷预热过后温度参数

// #define ERUPT_SHOOT_SPEED_FRONT   31350
// #define ERUPT_SHOOT_SPEED_BACK  22900//天气不太寒冷预热过后温度参数

// #define ERUPT_SHOOT_SPEED_FRONT   42500
// #define ERUPT_SHOOT_SPEED_BACK  33900//0度以下，未预热的温度度参数






void Shoot_init();
void Shoot_resolution();
void Shoot_PID();



class Class_Shoot{

private:




public:


    Class_Shoot(uint8_t STATE=0,float VELOCITY=0);
    uint8_t state_friction;//0表示摩擦轮关闭，1表示摩擦轮开启
    uint8_t state_plate;//0表示关闭，1表示点射，2表示连射
    float Delta_vel_shoot_plus;
    float Delta_vel_shoot_minus;
    float velocity;//摩擦轮的预期速度
    float velocity_planned;//摩擦轮转速规划
    float plate_velocity;//拨盘预期的位置，以后轮为标准
    float plate_velocity_plan;//拨盘规划的位置，防止速度过快，以后轮为标准
    float plate_locked;//表示plate是否发生了堵转，如果发生了堵转，则为1，如果没有，则为0，以后轮为标准
	uint16_t Locked_time;//堵转时间，以后轮为标准
	uint16_t Jamming_slove_time;//解决堵转时间，以后轮为标准
};




void velocity_plan_s(float velocity,float *velocity_plan,float Delta_plus,float Delta_minus);
void Bounce_speed_planning(void);
void magazine_open(void);
void magazine_close(void);

#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif 

#endif
