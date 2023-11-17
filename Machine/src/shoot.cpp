#include "moter.h"
#include "shoot.h"
#include "remote.h"
#include "rtthread.h"
#include <rthw.h>
#include "referee.h"
#include "Servo.h"



C620_driver C620_plate(0x02,3591.0f/187.0f);
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
    plate_location=0;
    plate_location_plan=0;
    velocity=VELOCITY;
    velocity_planned=0;
    plate_locked=0;
    
}





void Shoot_init(){

    C620_plate.location_PID.PID_init(12,0,5);
    C620_plate.location_PID.PID_anti_integ_saturated_init(720,-720);
    C620_plate.velocity_PID.PID_init(35*PI/9,0.4f*PI*Shoot_Tick/3,0);
    C620_plate.velocity_PID.PID_anti_integ_saturated_init(Current_MAX_M2006,-Current_MAX_M2006);



    
    friction_left_front.velocity_PID.PID_init(0.95f,0.0047f,2);
    friction_left_front.velocity_PID.PID_anti_integ_saturated_init(Current_MAX_M3508,-Current_MAX_M3508);

    friction_left_back.velocity_PID.PID_init(0.95f,0.0047f,1);
    friction_left_back.velocity_PID.PID_anti_integ_saturated_init(Current_MAX_M3508,-Current_MAX_M3508);

    
    friction_right_front.velocity_PID.PID_init(0.95f,0.0047f,2);
    friction_right_front.velocity_PID.PID_anti_integ_saturated_init(Current_MAX_M3508,-Current_MAX_M3508);

    friction_right_back.velocity_PID.PID_init(0.95f,0.0047f,1);
    friction_right_back.velocity_PID.PID_anti_integ_saturated_init(Current_MAX_M3508,-Current_MAX_M3508);

}



void Shoot_resolution(){
    
    
        if(Shoot_back.state_friction==0)//后置摩擦轮判断是否运行
        {
            
            velocity_plan_s(0,&Shoot_back.velocity_planned,100000*Shoot_Tick,100000*Shoot_Tick);
            friction_left_back.velocity_target=Shoot_back.velocity_planned;
            friction_right_back.velocity_target=-Shoot_back.velocity_planned;

        }
        else{//状态为1时开始运行
            
            velocity_plan_s(Shoot_back.velocity,&Shoot_back.velocity_planned,100000*Shoot_Tick,100000*Shoot_Tick);
            friction_left_back.velocity_target=Shoot_back.velocity_planned;
            friction_right_back.velocity_target=-Shoot_back.velocity_planned;

        }

        if(Shoot_front.state_friction==0)//前置摩擦轮判断是否运行
        {

            velocity_plan_s(0,&Shoot_front.velocity_planned,100000*Shoot_Tick,100000*Shoot_Tick);
            friction_left_front.velocity_target=Shoot_front.velocity_planned;
            friction_right_front.velocity_target=-Shoot_front.velocity_planned;

        }
        else
        {

            velocity_plan_s(Shoot_front.velocity,&Shoot_front.velocity_planned,100000*Shoot_Tick,100000*Shoot_Tick);
            friction_left_front.velocity_target=Shoot_front.velocity_planned;
            friction_right_front.velocity_target=-Shoot_front.velocity_planned;

        }


    //未进入回避时判断正向拨动受阻
	if((C620_plate.get_current_real()>9500)&&(Shoot_back.plate_locked == 0)&&(Shoot_front.plate_locked == 0))
	{
		Shoot_front.Locked_time++;
        Shoot_back.Locked_time++;
	}
	//受阻超过一定时间则进入处理逻辑
	if(Shoot_back.Locked_time >= 200)
	{
		Shoot_back.Jamming_slove_time++;
		if(Shoot_back.plate_locked == 0)
		{
			Shoot_back.plate_locked = 1;
			Shoot_back.plate_location = C620_plate.get_location_real() - 45.0f;
		}
	}
	//处理时间结束, 而后恢复正常
	if(Shoot_back.Jamming_slove_time >= 150)
	{
		Shoot_back.Locked_time = 0;
		Shoot_back.Jamming_slove_time = 0;
		Shoot_back.plate_locked = 0;
	}
		
        if(Shoot_back.plate_location>=Shoot_back.plate_location_plan){
            if(Shoot_back.plate_location-Shoot_back.plate_location_plan>Shoot_plate_delta_location){
                Shoot_back.plate_location_plan+=Shoot_plate_delta_location;
            }else{
                Shoot_back.plate_location_plan=Shoot_back.plate_location;
            }
        }else{
            if(Shoot_back.plate_location-Shoot_back.plate_location_plan<-Shoot_plate_delta_location){
                Shoot_back.plate_location_plan-=Shoot_plate_delta_location;
            }else{
                Shoot_back.plate_location_plan=Shoot_back.plate_location;
            }

        }
        C620_plate.location_target=Shoot_back.plate_location_plan;



        
                


}

void Shoot_PID(){

    friction_left_front.current_target=friction_left_front.velocity_PID.PID_anti_integral_saturated(friction_left_front.velocity_target,friction_left_front.get_velocity_real());
    friction_right_front.current_target=friction_right_front.velocity_PID.PID_anti_integral_saturated(friction_right_front.velocity_target,friction_right_front.get_velocity_real());
    friction_left_back.current_target=friction_left_back.velocity_PID.PID_anti_integral_saturated(friction_left_back.velocity_target,friction_left_back.get_velocity_real());
    friction_right_back.current_target=friction_right_back.velocity_PID.PID_anti_integral_saturated(friction_right_back.velocity_target,friction_right_back.get_velocity_real());

    C620_plate.velocity_target=C620_plate.location_PID.PID_anti_integral_saturated(C620_plate.location_target,C620_plate.get_location_real());
    C620_plate.current_target=C620_plate.velocity_PID.PID_anti_integral_saturated(C620_plate.velocity_target,C620_plate.get_velocity_real());
    
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

/*高校联盟赛弹速规划*/
void Bounce_speed_planning(void)
{
	if(Referee.Game_robot_status.shooter_id1_42mm_speed_limit == 16)//弹速限制
	{	
        Shoot_front.velocity = ERUPT_SHOOT_SPEED_FRONT;
		Shoot_back.velocity = ERUPT_SHOOT_SPEED_BACK;//爆发优先	
	}
	else if(Referee.Game_robot_status.shooter_id1_42mm_speed_limit == 16)
	{
        Shoot_front.velocity = SPEED_SHOOT_SPEED_FRONT;
		Shoot_back.velocity = SPEED_SHOOT_SPEED_BACK;//弹速优先
	}
    else //低保
    {
        Shoot_front.velocity = ERUPT_SHOOT_SPEED_FRONT;
        Shoot_back.velocity = ERUPT_SHOOT_SPEED_BACK;//爆发优先
    }
}

//开弹仓
void magazine_open(void)
{
	set_servo_revolve_angle(100);
}

//关弹仓
void magazine_close(void)
{
	set_servo_revolve_angle(45);
}

