#ifndef CHASSIS_H
#define CHASSIS_H

#ifdef __cplusplus       //-----------��׼д��-----------
extern "C"{                      //-----------��׼д��-----------
#endif
	
	
	
	
#include "moter.h"
#include "PID.h"

#define wheel_length  0.427f
#define wheel_width   0.394f
#define wheel_distance 0.3f

#define state_normal 1
#define state_ackerman 2
#define state_gyro   3

#define Super_Cap_warning_energy  (60.0f)//超级电容剩余的电量最低值


#define Chassis_Tick  (3u)



#define Chassis_GM6020_Delta_location  (0.54f*Chassis_Tick)









class Class_Chassis{

private:
    float velocity_x_planned;
    float velocity_y_planned;
    float velocity_angle_planned;


    float Delta_vel_x_plus;
    float Delta_vel_y_plus;
    float Delta_vel_angle_plus;//表示每�?�底盘任务执行的时候速度的变化量，与底盘任务的执行周期密切相关，如果�?标速度和实际速度相差过大，会按照�?形曲线的方式增加

    float Delta_vel_x_minus;
    float Delta_vel_y_minus;
    float Delta_vel_angle_minus;


    //float Power;//底盘当前的实际功率，从电源管理模块中获得
    //float Power_Max;//底盘当前的最大功率，从主控模块里面获得
    //float Power_Left;//扣除掉6020占用功率后给3508驱动的电机功率
    //float Super_cap_energy;//超级电容剩余的能量
    //float C620_current_max;//底盘3508电流的最大值，用来限制底盘的功率
    



public:
    
    float velocity_x;
    float velocity_y;
    float velocity_angle;
    uint8_t state;//底盘的状态

    float K_vel;//底盘的增益系数

    int8_t zeroing_state;//表明底盘是否处在归零模式，如果处在归零模式，该变量置为1，不在，则该变量放置为0
    Class_PID PID_angle;//表明底盘由速度模式到位置模式的PID
    float angle_target;//表示的是我希望转到的一个目标位置，来源于Yaw6020的返回值
    float angle_init;//表示该刚刚进入回零模式的时候底盘从陀螺仪处获得到的Yaw角度值



    float wheel_angle;//在三轮车模式下前轮的摆动的角度，幅度，仅在三轮车模式下有用
//    float angle_target1_planned;
//    float angle_target2_planned;
//    float angle_target3_planned;
//    float velocity_target1_planned;
//    float velocity_target2_planned;
//    float velocity_target3_planned;


    void Chassis_zeroing_if_OK();
    Class_Chassis();

    Class_PID Power_PID;

    void Chassis_init(float DELTA_VEL_X_PLUS,float DELTA_VEL_Y_PLUS,float DELTA_VEL_ANGLE_PLUS,float DELTA_VEL_X_MINUS,float DELTA_VEL_Y_MINUS,float DELTA_VEL_ANGLE_MINUS);
    void Chassis_Movement_Plan();





    float get_velocity_x_planned(){return velocity_x_planned;}
    void set_velocity_x_planned(float VELOCITY){velocity_x_planned=VELOCITY;}
    
    float get_velocity_y_planned(){return velocity_y_planned;}
    void set_velocity_y_planned(float VELOCITY){velocity_y_planned=VELOCITY;}


    float get_velocity_angle_planned(){return velocity_angle_planned;}
    void set_velocity_angle_planned(float VELOCITY){velocity_angle_planned=VELOCITY;}

    //float get_Power(){return Power;}
    //void set_Power(float POWER){Power=POWER;}

    //float get_Power_Max(){return Power_Max;}
    //void set_Power_Max(float POWER_Pax){Power_Max=POWER_Pax;}    

};


extern uint32_t adc_buf;


class Class_Super_Cup{

private:
    uint16_t CAN_ID;//CAN_ID
    CAN_HandleTypeDef *hcanp;
    float Chassis_Power_real;//底盘此时真实消耗的功率，包括电源供电和超级电容放电，单位为瓦特，W
    float Energy_rest;//超级电容底盘的剩余能量，单位为焦耳J
    int16_t Power_Limit;//发送底盘的限制功率，功率为W
public:
    int16_t Power_Limit_Max;//底盘被限制住的最大功率
    Class_Super_Cup(CAN_HandleTypeDef *HCANP,uint16_t ID){hcanp=HCANP;CAN_ID=ID;};
    Class_Super_Cup(CAN_HandleTypeDef *HCANP,uint16_t ID,int16_t POWER_LIM_MAX){hcanp=HCANP;CAN_ID=ID;Power_Limit_Max=POWER_LIM_MAX;};
    void set_Energy_rest(float ENERGY_REST){Energy_rest=ENERGY_REST;}
    float get_Energy_rest(){return Energy_rest;}
    void set_Chassis_Power_real(float POWER_REAL){ Chassis_Power_real=POWER_REAL;}
    float get_Chassis_power_real(){return Chassis_Power_real;}
    void Can_Data_processing(uint8_t buf[]);
    void Send_PowerLimt(int16_t Power_l);
    void Send_PowerLimt();
};





void Chassis_Power_Limit();
void Chassis_resolution_classic();

void Chassis_Mecanum_wheel_init();

void Chassis_Mecanum_wheel_resolution();

void Chassis_Mecanum_wheel_PID();


void velocity_plan_c(float velocity,float *velocity_plan,float Delta_plus,float Delta_minus);
void location_plan_c(float location,float *location_plan,float Delta_location);






























#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif 


#endif
