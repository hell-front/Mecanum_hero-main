#include "moter.h"
#include "shoot.h"
#include "remote.h"
#include "rtthread.h"
#include <rthw.h>
#include "referee.h"
#include "Servo.h"



C620_driver C620_Plate(0x02);
C620_driver friction_left_front(0x04,1);
C620_driver friction_right_front(0x03,1);
C620_driver friction_left_back(0x08,1);
C620_driver friction_right_back(0x07,1);



//extern Remote_data Remote;

//struct  shoot_move shoot={1};


Class_Shoot Shoot_front(1,ERUPT_SHOOT_SPEED_FRONT);
Class_Shoot Shoot_back(1,ERUPT_SHOOT_SPEED_BACK);


extern Referee_System Referee;

float Firing_frequency = 1.2f ;//低保射频 

Class_Shoot::Class_Shoot(uint8_t STATE,float VELOCITY){
    state_friction=STATE;
    state_plate=2;
    plate_velocity=0;
    plate_velocity_plan=0;
    velocity=VELOCITY;
    velocity_planned=0;
    plate_locked=0;
    
}





void Shoot_init(){

    // C620_Plate.location_PID.PID_init(12.0f,0,5.0f);
    // C620_Plate.location_PID.PID_anti_integ_saturated_init(720,-720);
    C620_Plate.velocity_PID.PID_init(12,0.5f,8);
    C620_Plate.velocity_PID.PID_anti_integ_saturated_init(Current_MAX_M3508,-Current_MAX_M3508);



    
    friction_left_front.velocity_PID.PID_init(5.0f,0.0047f,4.0f);
    friction_left_front.velocity_PID.PID_differ_filter_init(0.3f);
    friction_left_front.velocity_PID.PID_anti_integ_saturated_init(Current_MAX_M3508,-Current_MAX_M3508);

    friction_left_back.velocity_PID.PID_init(5.0f,0.0047f,4.0f);
    friction_left_back.velocity_PID.PID_differ_filter_init(0.3f);
    friction_left_back.velocity_PID.PID_anti_integ_saturated_init(Current_MAX_M3508,-Current_MAX_M3508);

    
    friction_right_front.velocity_PID.PID_init(5.0f,0.0047f,4.0f);
    friction_right_front.velocity_PID.PID_differ_filter_init(0.3f);
    friction_right_front.velocity_PID.PID_anti_integ_saturated_init(Current_MAX_M3508,-Current_MAX_M3508);

    friction_right_back.velocity_PID.PID_init(5.0f,0.0047f,4.0f);
    friction_right_back.velocity_PID.PID_differ_filter_init(0.3f);
    friction_right_back.velocity_PID.PID_anti_integ_saturated_init(Current_MAX_M3508,-Current_MAX_M3508);

}



void Shoot_resolution(){
    
    
        if(Shoot_back.state_friction==0)//后置摩擦轮判断是否运行
        {
            
            // velocity_plan_s(0,&Shoot_back.velocity_planned,100000*Shoot_Tick,100000*Shoot_Tick);
            friction_left_back.velocity_target=0;
            friction_right_back.velocity_target=0;

            // velocity_plan_s(0,&Shoot_front.velocity_planned,100000*Shoot_Tick,100000*Shoot_Tick);
            friction_left_front.velocity_target=0;
            friction_right_front.velocity_target=0;

        }
        else{//状态为1时开始运行
            
            // velocity_plan_s(Shoot_back.velocity,&Shoot_back.velocity_planned,100000*Shoot_Tick,100000*Shoot_Tick);
            friction_left_back.velocity_target=Shoot_back.velocity;
            friction_right_back.velocity_target=-Shoot_back.velocity;

            // velocity_plan_s(Shoot_front.velocity,&Shoot_front.velocity_planned,100000*Shoot_Tick,100000*Shoot_Tick);
            friction_left_front.velocity_target=Shoot_front.velocity;
            friction_right_front.velocity_target=-Shoot_front.velocity;

        }


    //未进入回避时判断正向拨动受阻,先不写
	if((C620_Plate.get_current_real()>9500)&&(Shoot_back.plate_locked == 0)&&(Shoot_front.plate_locked == 0))
	{
        Shoot_back.Locked_time++;
	}
	//受阻超过一定时间则进入处理逻辑
	if(Shoot_back.Locked_time >= 200)
	{
		Shoot_back.Jamming_slove_time++;
		if(Shoot_back.plate_locked == 0)
		{
			Shoot_back.plate_locked = 1;
			// Shoot_back.plate_location = C620_Plate.location_real - 45.0f;
		}
	}
	//处理时间结束, 而后恢复正常
	if(Shoot_back.Jamming_slove_time >= 150)
	{
		Shoot_back.Locked_time = 0;
		Shoot_back.Jamming_slove_time = 0;
		Shoot_back.plate_locked = 0;
	}
    if(friction_left_back.velocity_real<=(ERUPT_SHOOT_SPEED_BACK-2000))//判断后方电机是否发射弹丸
    {
        Shoot_back.plate_velocity=0;
    }
        C620_Plate.velocity_target=Shoot_back.plate_velocity;

}

void Shoot_PID(){

    friction_left_front.current_target=friction_left_front.velocity_PID.PID_anti_integral_saturated(friction_left_front.velocity_target,friction_left_front.get_velocity_real());
    friction_right_front.current_target=friction_right_front.velocity_PID.PID_anti_integral_saturated(friction_right_front.velocity_target,friction_right_front.get_velocity_real());
    friction_left_back.current_target=friction_left_back.velocity_PID.PID_anti_integral_saturated(friction_left_back.velocity_target,friction_left_back.get_velocity_real());
    friction_right_back.current_target=friction_right_back.velocity_PID.PID_anti_integral_saturated(friction_right_back.velocity_target,friction_right_back.get_velocity_real());

    // C620_Plate.velocity_target=C620_Plate.location_PID.PID_absolute(C620_Plate.location_target,C620_Plate.location_real);

    C620_Plate.current_target=C620_Plate.velocity_PID.PID_absolute(C620_Plate.velocity_target,C620_Plate.velocity_real);
    if(Shoot_back.plate_velocity==0)//判断后方电机是否发射弹丸
    {
        C620_Plate.current_target=0;
    }
}

void velocity_plan_s(float velocity,float *velocity_plan,float Delta_plus,float Delta_minus){

    if(((velocity)*(*velocity_plan)>=0)&&((velocity)!=0)){
        if(fabsf(velocity)-fabsf(*velocity_plan)>=0){
            if(fabsf(velocity)-fabsf(*velocity_plan)<=Delta_plus){
                *velocity_plan=velocity;
            }else{
                *velocity_plan+=(velocity>0)?Delta_plus:-Delta_plus;
            }
        }
    }else if(velocity==0){
        if(fabsf(*velocity_plan)<Delta_minus){
            *velocity_plan=0;
        }else{
            *velocity_plan+=(*velocity_plan>0)?(-Delta_minus):Delta_minus;
        }
    }else{
        if(fabsf(*velocity_plan)<Delta_minus){
            *velocity_plan=0;
        }else{
            *velocity_plan+=(*velocity_plan>0)?(-Delta_minus):Delta_minus;
        }
    }

    


}

