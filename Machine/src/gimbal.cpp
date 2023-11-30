#include "gimbal.h"
#include "main.h"
#include "Chassis.h"
#include "Ablock.h"
#include"imu_mini.h"
#include "Pan_Tilt_AHRS.h"
#include "imu_mini.h"
DM_motor DM4310_pitch(0x10,0.0f);
DM_motor DM6006_yaw(0x10,0.0f);

extern Class_Chassis Chassis;


#define PITCH_BALANCE_CURRENT -6550

Class_Gimbal Gimbal(0,0);


extern Class_Remote_data Remote;

//float YAW;//表示云台相�?�其0方向偏航多少
//extern Class_MPU6500 Mpu_6500;
extern Class_imu_mini Imu_mini;

uint8_t auto_aim_buf[20];



extern float aaa;





uint32_t zeroing_time=0;//表示电机进入阈值的时间，



void Gimbal_init()//该函数主要用于处理设置pitch轴和yaw轴电机的pid参数
{
        DM4310_pitch.location_max = 70.0f;
        DM4310_pitch.location_min = 30.0f;


        Gimbal.location_pitch=0;
        Gimbal.location_yaw=0;
        Gimbal.yaw_real=0;
        
        Gimbal.yaw_target=0;
        Gimbal.pitch_target=0;
        Gimbal.yaw_comp=0;
        Gimbal.pitch_comp=0;
        

        float yaw_now;
        yaw_now=DM6006_yaw.location_real;

        //Gimbal.PID_pitch.PID_init(0.5f,0,0.0f/Gimbal_Tick);//表示的是用于自瞄的PID参数的初始化，理论上直接等于位置环的PID即可，如果不稳则可以通过该小参数来解决
        Gimbal.PID_yaw.PID_init(0.5f,0,0.0f/Gimbal_Tick);
        Gimbal.PID_gyro.PID_init(6.5f,0.00001,105.4f/Gimbal_Tick);//表示用于小陀螺计算的时候的pid

        //电机pid参数设置，包括Kp，Ki，Kd,过滤器比例，结果的取值范围

        // DM6006_yaw.location_PID.PID_init(20.5,0,10.0f/Gimbal_Tick);
        DM6006_yaw.location_PID.PID_init(20.5,0,3.0f);
        DM6006_yaw.location_PID.PID_differ_filter_init(0.3f);
        DM6006_yaw.location_PID.PID_anti_integ_saturated_init(360.0f,-360.0f);

        // DM6006_yaw.velocity_PID.PID_init(1.0f*1000*PI/18.0f,5.5f*PI*Gimbal_Tick/18.0f,4000.0f*PI/(54.0f*Gimbal_Tick));
        // DM6006_yaw.velocity_PID.PID_init(174.4f,2.88f,77.5f);
        // DM6006_yaw.velocity_PID.PID_differ_filter_init(0.03f);
        // DM6006_yaw.velocity_PID.PID_anti_integ_saturated_init(3,-3);


        //电机pid参数设置，包括Kp，Ki，Kd,过滤器比例，结果的取值范围

        // DM4310_pitch.location_PID.PID_init(18,0,40.0f/Gimbal_Tick);
        DM4310_pitch.location_PID.PID_init(8,0.02,-0.5);
        DM4310_pitch.location_PID.PID_differ_filter_init(0.03f);
        DM4310_pitch.location_PID.PID_anti_integ_saturated_init(-1000.0f,1000.0f);


        if(DM6006_yaw.location_real>185.0f)
        {
                DM6006_yaw.location_zero=365.0f;
        }
  
        if(DM4310_pitch.location_real>180.0f)
        {
                DM4310_pitch.location_zero=360.0f;
        }


        Gimbal.pitch_planned=DM4310_pitch.location_real-DM4310_pitch.location_zero;
        Gimbal.yaw_planned=DM6006_yaw.location_real-DM6006_yaw.location_zero;

}

void Gimbal_resloution()//该函数主要用于初始化
{

        static float angle_z_last=0;
        float angle_z;
        
        angle_z=-Imu_mini.Angle_Yaw;
        //读取底部陀螺仪的参数作为yaw轴补偿角度，用在小陀螺模式



        Gimbal.yaw_real=fmodf(DM6006_yaw.location_real-DM6006_yaw.location_zero,360.0f);//表示的获取此时的云台相对电机的转动角度


        if(Gimbal.yaw_real>180.0f&&Gimbal.yaw_real<360.0f)//表示的是将角度转换为-180度到180度的区间内
        {
                Gimbal.yaw_real-=360.0f;
        }
        else if (Gimbal.yaw_real<-180.0f&&Gimbal.yaw_real>-360.0f)
        {
                Gimbal.yaw_real+=360.0f;
        }



        if(Gimbal.gimbal_auto==1)//表示是否开启了自动瞄准，开启了则要对云台两个电机的角度进行一个补偿
        {

                Gimbal.location_yaw+=0.001*Gimbal_Tick*Gimbal.PID_yaw.PID_absolute(0,-Gimbal.yaw_comp);//相对于当前位置去拟合，假定planed为当前位置以规避噪声


        }


        if(Chassis.state==state_gyro)//表示是否进入了小陀螺要对云台位置进行改变
        {
                
                Gimbal.location_yaw+=angle_z_last-angle_z;//此为通常做法，此做法配合PD控制不可避免的存在偏移

        }
        else
        {//表明此时底盘处在正常模式，需要判定是否处在归零模式
                if((Gimbal.yaw_real>5.0f||Gimbal.yaw_real<-5.0f))//判定此时是否需要放置未归零模式，此时需要未归零模式
                {
                        if(Gimbal.yaw_real > 5.0f)
                        Chassis.zeroing_state=1;
                        else Chassis.zeroing_state=-1;

                        
                        Chassis.angle_target=Gimbal.yaw_real;//表示云台偏差的在-180~180°的单圈值，由电机获取
                        Chassis.angle_init=-Imu_mini.Angle_Yaw_real;//表示的是底盘此刻的多圈角度值，由陀螺仪获取
                }
        }
        if(Chassis.state==state_normal && Chassis.zeroing_state)
        {
                Gimbal.location_yaw+=angle_z_last-angle_z;//时刻减去底盘此时相对初始值转过的角度
        }
        // if(Gimbal.yaw_real<1.0f&&Gimbal.yaw_real>-1.0f&&Chassis.zeroing_state&&Chassis.state==state_normal){//如果误差小于一定范围且要在一定时间，则认为置零过程结束
        //         if(zeroing_time>=2){
        //                 Chassis.zeroing_state=0;
        //                 Chassis.velocity_angle=0;//结束的时候将底盘角速度赋值为0
        //         }else{
        //                 zeroing_time++;
        //         }

        // }else{
        //         zeroing_time=0;
        // }
        if(Chassis.zeroing_state == -1)
        {
                if (Gimbal.yaw_real > 0)
                {
                        Chassis.zeroing_state=0;
                        Chassis.velocity_angle=0;//结束的时候将底盘角速度赋值为0
                }        
        }
        else if(Chassis.zeroing_state == 1){
                if (Gimbal.yaw_real < 0)
                {
                        Chassis.zeroing_state=0;
                        Chassis.velocity_angle=0;//结束的时候将底盘角速度赋值为0
                }        
        }


        Gimbal.yaw_target=Gimbal.location_yaw;//表示得到最终的目标角度
        Gimbal.pitch_target=Gimbal.location_pitch;
        


        if(Gimbal.Init_OK&&Chassis.state==state_gyro){
                Gimbal.Delta_Pitch_Max=Gimbal_Delta_Pitch;
                Gimbal.Delta_Yaw_Max=Gimbal_Delta_Yaw;

        }else if(Gimbal.Init_OK){
                Gimbal.Delta_Pitch_Max=Gimbal_Delta_Pitch;
                Gimbal.Delta_Yaw_Max=0.6f*Gimbal_Delta_Yaw;

        }
        location_plan_g(Gimbal.pitch_target,&Gimbal.pitch_planned,Gimbal.Delta_Pitch_Max);//对云台进行速度规划
        location_plan_g(Gimbal.yaw_target,&Gimbal.yaw_planned,Gimbal.Delta_Yaw_Max);


        Gimbal.gimbal_auto_last=Gimbal.gimbal_auto;//表示的是上一次的云台状态的值

        DM4310_pitch.location_target=Gimbal.pitch_planned+DM4310_pitch.location_zero;
        DM6006_yaw.location_target=Gimbal.yaw_planned+DM6006_yaw.location_target;

        //aaa=angle_z;

        angle_z_last=angle_z;

}


void Gimbal_PID(){
        uint8_t chassis_gryo=Chassis.state;
        static uint8_t chassis_gryo_last=chassis_gryo;//该变量存储的是上一时刻底盘是否进入了小陀螺状态，和这次对比用以清除Yaw的PID参数
        if(chassis_gryo!=chassis_gryo_last){
                DM6006_yaw.location_PID.PID_clear();
                if(chassis_gryo==3){//表示这时候进入底盘进入了小陀螺状态，PID参数换成进入小陀螺状态的参数
                        DM6006_yaw.location_PID.PID_init(8.5,0,60.0f/Gimbal_Tick);//该参数可能要进行修改   
                }else{//表示底盘此时不在小陀螺状态，参数要换成不是小陀螺状态的参数
                        DM6006_yaw.location_PID.PID_init(8.5,0,60.0f/Gimbal_Tick);  
                }
        }
        chassis_gryo_last=chassis_gryo;


        // GM6020_pitch.velocity_target=GM6020_pitch.location_PID.PID_differ_filter_anti_saturated(GM6020_pitch.location_target,GM6020_pitch.get_location_real());
        // if(Chassis.state==state_gyro||Chassis.zeroing_state){//表示的是进入了小陀螺状态或者是进入归零模式
        //         GM6020_yaw.velocity_target=GM6020_yaw.location_PID.PID_differ_filter_anti_saturated(GM6020_yaw.location_target,GM6020_yaw.get_location_real());
        //        GM6020_yaw.velocity_target=-1.1f*57.2957805f*Imu_mini.Yaw_speed_real+GM6020_yaw.location_PID.PID_differ_filter_anti_saturated(GM6020_yaw.location_target,GM6020_yaw.get_location_real());
        //         第二个函数表示的是加上前馈控制的函数，可能要重新计算PID
        // }else{//表示的是没有进入小陀螺状态
        //        GM6020_yaw.velocity_target=GM6020_yaw.location_PID.PID_differ_filter_anti_saturated(GM6020_yaw.location_target,GM6020_yaw.get_location_real());
        // }
        // if(Chassis.state!=chassis_gryo&&Gimbal.gimbal_auto==0){//表示没有进入小陀螺状态也没有进入自瞄状态
        DM4310_pitch.velocity_target=DM4310_pitch.location_PID.PID_absolute(DM4310_pitch.location_target,DM4310_pitch.location_real);
                // DM6006_yaw.velocity_target=DM6006_yaw.location_PID.PID_differ_filter_anti_saturated(DM6006_yaw.location_target,DM6006_yaw.location_real);
                // DM4310_pitch.velocity_target=DM4310_pitch.location_PID.PID_differ_filter_anti_saturated(DM4310_pitch.location_target,DM4310_pitch.location_real);
        // }else if(Chassis.state==chassis_gryo&&Gimbal.gimbal_auto==0){//表示进入了小陀螺状态但是没有进入自瞄状态
        //         DM6006_yaw.velocity_target=-57.2957805f*Imu_mini.Yaw_speed_real+DM6006_yaw.location_PID.PID_differ_filter_anti_saturated(DM6006_yaw.location_target,DM6006_yaw.location_real);
        //         DM4310_pitch.velocity_target=DM4310_pitch.location_PID.PID_differ_filter_anti_saturated(DM4310_pitch.location_target,DM4310_pitch.location_real);
        // }else if(Chassis.state!=chassis_gryo&&Gimbal.gimbal_auto){//表示没有进入小陀螺但是进入了自瞄状态
        //         DM6006_yaw.velocity_target=-Gimbal.Predicted_Velocity_Yaw/100.0f+DM6006_yaw.location_PID.PID_differ_filter_anti_saturated(DM6006_yaw.location_target,DM6006_yaw.location_real);
        //         DM4310_pitch.velocity_target=Gimbal.Predicted_Velocity_Pitch/100.0f+DM4310_pitch.location_PID.PID_differ_filter_anti_saturated(DM4310_pitch.location_target,DM4310_pitch.location_real);
        // }else{//表示进入了进入了小陀螺也进入了自瞄状态
        //         DM6006_yaw.velocity_target=-Gimbal.Predicted_Velocity_Yaw/100.0f-57.2957805f*Imu_mini.Yaw_speed_real+DM6006_yaw.location_PID.PID_differ_filter_anti_saturated(DM6006_yaw.location_target,DM6006_yaw.location_real);
        //         DM4310_pitch.velocity_target=Gimbal.Predicted_Velocity_Pitch/100.0f+DM4310_pitch.location_PID.PID_differ_filter_anti_saturated(DM4310_pitch.location_target,DM4310_pitch.location_real);
        // }



        // //GM6020_pitch.current_target=GM6020_pitch.velocity_PID.PID_differ_filter_anti_saturated(GM6020_pitch.velocity_target,GM6020_pitch.get_velocity_real()) + PITCH_BALANCE_CURRENT;
        // // GM6020_pitch.current_target=GM6020_pitch.velocity_PID.PID_differ_filter_anti_saturated(GM6020_pitch.velocity_target,GM6020_pitch.get_velocity_real());
        // //根据陀螺仪的安装方式，pitch轴对应的是陀螺仪的yaw
        // GM6020_pitch.current_target=GM6020_pitch.velocity_PID.PID_differ_filter_anti_saturated(GM6020_pitch.velocity_target,Pan_Tilt_AHRS.GetYawOmega());
        // // GM6020_yaw.current_target=GM6020_yaw.velocity_PID.PID_differ_filter_anti_saturated(GM6020_yaw.velocity_target,GM6020_yaw.get_velocity_real());
        // GM6020_yaw.current_target=GM6020_yaw.velocity_PID.PID_differ_filter_anti_saturated(GM6020_yaw.velocity_target,-Pan_Tilt_AHRS.GetPitchOmega()  - Imu_mini.Yaw_speed_real * 180 / PI);
        
        // GM6020_yaw.voltage = 0;
        // GM6020_pitch.voltage=GM6020_pitch.current_PID.PID_anti_integral_saturated(GM6020_pitch.current_target,GM6020_pitch.get_current_real());
        // GM6020_yaw.voltage=GM6020_yaw.current_PID.PID_anti_integral_saturated(GM6020_yaw.current_target,GM6020_yaw.get_current_real());
        
}



void location_plan_g(float location,float *location_plan,float Delta_location){
        if(location-*location_plan>0){
                if(location-*location_plan<Delta_location){
                        *location_plan=location;
                }else{
                        *location_plan+=Delta_location;
                }
        }else{
                if(location-*location_plan>-Delta_location){
                        *location_plan=location;
                }else{
                        *location_plan-=Delta_location;
                }
        }
}

void Class_Gimbal::UART_Data_processing(uint8_t buf[]){
        memcpy(&Frame_Header,auto_aim_buf,2);
        memcpy(&Delta_Yaw,auto_aim_buf+2,2);
        memcpy(&Delta_pitch,auto_aim_buf+4,2);
        memcpy(&Predicted_Delta_Yaw,auto_aim_buf+6,2);
        memcpy(&Predicted_Delta_Pitch,auto_aim_buf+8,2);
        memcpy(&Distance,auto_aim_buf+10,2);
        memcpy(&Predicted_Velocity_Yaw,auto_aim_buf+12,2);
        memcpy(&Predicted_Velocity_Pitch,auto_aim_buf+14,2);
        memcpy(&Manifold_Flag,auto_aim_buf+16,2);

        yaw_comp=Predicted_Delta_Yaw/100.0f;
        pitch_comp=Predicted_Delta_Pitch/100.0f;

        
}
