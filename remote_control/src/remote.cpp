#include "main.h"
#include "Chassis.h"
#include "gimbal.h"
#include "remote.h"
#include "shoot.h"
#include "referee.h"





uint8_t remote_buf[18];


extern struct Class_Chassis Chassis;
extern Class_Gimbal Gimbal;
extern Class_Shoot Shoot_front;
extern Class_Shoot Shoot_back;
extern DM_motor DM4310_pitch;
extern DM_motor DM6006_yaw;

extern Class_Super_Cup Super_Cup;
extern Referee_System Referee;
extern float Firing_frequency;


//Class_Remote_data Remote={1024,1024,1024,1024,1,1,0,0,0,0,0,0,1024};
Class_Remote_data Remote;

int32_t Shoot_speed_flag = 0;//弹速标志
int16_t test_flag = 0; 
uint8_t magazine_open_flag = 0;
uint8_t gyro_state_flag_left = 0;
uint8_t gyro_state_flag_right = 0;
int16_t mouse_x_value = 0;
int16_t mouse_y_value = 0;

Class_Remote_data::Class_Remote_data(){
        Channel_0=1024;
        Channel_1=1024;
        Channel_2=1024;
        Channel_3=1024;
        Channel_user=1024;
        State_left=Remote_Switch_Left1;
        State_left_last=Remote_Switch_Left1;
        State_right1_num=0;
        State_right=Remote_Switch_Right2;
        State_right_last=Remote_Switch_Right2;
        Mouse_x=0;
        Mouse_y=0;
        Mouse_z=0;
        Mouse_left=0;
        Mouse_right=0;

        keyboard=0;
        connected=0;
        update=0;//假定上电时候遥控器未连接

}


/**
 * @brief 处理遥控器数据
 * 
 */
void Class_Remote_data::remote_data_processing()
{
        
        update++;

        Channel_0=((int16_t)remote_buf[0] | ((int16_t)remote_buf[1] << 8)) & 0x07FF;
        Channel_1=(((int16_t)remote_buf[1] >> 3) | ((int16_t)remote_buf[2] << 5))& 0x07FF;
        Channel_2=(((int16_t)remote_buf[2] >> 6) | ((int16_t)remote_buf[3] << 2) |((int16_t)remote_buf[4] << 10)) & 0x07FF;
        Channel_3=(((int16_t)remote_buf[4] >> 1) | ((int16_t)remote_buf[5]<<7)) &0x07FF;
        Channel_user=(((int16_t)remote_buf[17])<<8)|((int16_t)remote_buf[16]);


        State_left=((remote_buf[5] >> 4) & 0x000C) >> 2;
        State_right=((remote_buf[5] >> 4)&0x0003);


        Mouse_x= ((int16_t)remote_buf[6]) | ((int16_t)remote_buf[7] << 8);
        Mouse_y= ((int16_t)remote_buf[8]) | ((int16_t)remote_buf[9] << 8);
        Mouse_z=((int16_t)remote_buf[10]) | ((int16_t)remote_buf[11] << 8); 
        Mouse_left=remote_buf[12];
        Mouse_right=remote_buf[13];
        keyboard=((int16_t)remote_buf[14])| ((int16_t)remote_buf[15] << 8);
        Key_W=(uint8_t)(keyboard&0x01);
        Key_S=(uint8_t)((keyboard>>1)&0x01);
        Key_A=(uint8_t)((keyboard>>2)&0x01);
        Key_D=(uint8_t)((keyboard>>3)&0x01);
        Key_Shift=(uint8_t)((keyboard>>4)&0x01);
        Key_Ctrl=(uint8_t)((keyboard>>5)&0x01);
        Key_Q=(uint8_t)((keyboard>>6)&0x01);
        Key_E=(uint8_t)((keyboard>>7)&0x01);
        Key_R=(uint8_t)((keyboard>>8)&0x01);
        Key_F=(uint8_t)((keyboard>>9)&0x01);
        Key_G=(uint8_t)((keyboard>>10)&0x01);
        Key_Z=(uint8_t)((keyboard>>11)&0x01);
        Key_X=(uint8_t)((keyboard>>12)&0x01);
        Key_C=(uint8_t)((keyboard>>13)&0x01);
        Key_V=(uint8_t)((keyboard>>14)&0x01);
        Key_B=(uint8_t)((keyboard>>15)&0x01);



        if(State_left!=Remote_Switch_Left3){//表示这时候处于遥控器控制模式
                remote_DT7_control();

        }else{//表示这时候是出于客户端(键盘控制模式)
                //注意，底盘的模式改变没有写，目前默认底盘的模式为模式1，即正常模式
                remote_keyboard_control();
                

        }
       
        State_left_last=State_left;
        State_right_last=State_right;
        Mouse_right_last = Mouse_right;//传递上次右键值
	Mouse_left_last = Mouse_left;
	Key_shift_last = Key_Shift;
	Key_ctrl_last = Key_Ctrl;
	Key_X_last = Key_X;
	Key_Z_last = Key_Z;
	Key_R_last = Key_R;
        Key_Q_last = Key_Q;
        Key_E_last = Key_E;
        Key_F_last = Key_F;
        
        // if(Mouse_x>200)
        // {
        //         mouse_x_value = Mouse_x_last;
        // }
        // else if(Mouse_x < -200)
        // {
        //         mouse_x_value = Mouse_x_last;
        // }
        // else
        // {
        //         mouse_x_value = Mouse_x;
        // }

        // if(Mouse_y>80)
        // {
        //         mouse_y_value = Mouse_y_last;
        // }
        // if(Mouse_y < -80)
        // {
        //         mouse_y_value = Mouse_y_last;
        // }
        // else
        // {
        //         mouse_y_value = Mouse_y;
        // }

        mouse_x_value = Mouse_x;
        mouse_y_value = Mouse_y;
        Mouse_x_last = mouse_x_value;
        Mouse_y_last = mouse_y_value;
}

void Class_Remote_data::remote_DT7_control()//用遥控器控制机器人的函数
{


        Chassis.state=Remote.State_left;
        Chassis.velocity_x=(Channel_2-1024)*Gain_x;
        Chassis.velocity_y=(Channel_3-1024)*Gain_y;
        Chassis.velocity_angle=(Channel_0-1024)*Gain_omega;

//以下主要是对左侧开关进行分析
        if(State_left==Remote_Switch_Left1||State_left==Remote_Switch_Left2)//当档位为1和2时，开启底盘和云台，关闭shoot
        {
                
                Chassis.velocity_x=(Channel_2-1024)*Gain_x;
                Chassis.velocity_y=(Channel_3-1024)*Gain_y;
                Chassis.velocity_angle=(Channel_0-1024)*Gain_omega;
                
                Gimbal.location_yaw+=(Channel_user-1024)*Gain_yaw;
                Gimbal.location_pitch+=(Channel_1-1024)*Gain_pitch;
                if(Gimbal.location_pitch<DM4310_pitch.location_min)
                {
                        Gimbal.location_pitch=DM4310_pitch.location_min;
                }
                else if(Gimbal.location_pitch>DM4310_pitch.location_max)
                {
                        Gimbal.location_pitch=DM4310_pitch.location_max;
                }

        }else if(State_left==Remote_Switch_Left3)//当档位为3时，仅开启yaw和y方向，开启shoot
        {
                
                //目前不太清楚为什么使用channel_user作为开启shoot的依据
                Gimbal.location_yaw+=-(Channel_0-1024)*Gain_yaw;
                Chassis.velocity_y=(Channel_3-1024)*Gain_y;
                // Chassis.wheel_angle=-(Channel_2-1024)*0.16f;
                // if(Chassis.wheel_angle>60.0f){
                //         Chassis.wheel_angle=60.0f;
                // }else if(Chassis.wheel_angle<-60.0f){
                //         Chassis.wheel_angle=-60.0f;
                // }


                Shoot_back.velocity+=(Channel_user-1024)*Gain_friction;//开始对后摩擦轮进行加速
                Shoot_front.velocity+=(Channel_user-1024)*Gain_friction;//开始对前摩擦轮进行加速

                //具体的加速过程，写在shoot的 Shoot_resolution() 函数中

                if (Shoot_back.velocity>ERUPT_SHOOT_SPEED_BACK)//设定后轮速度上下限
                {
                        Shoot_back.velocity=ERUPT_SHOOT_SPEED_BACK;
                }
                else if (Shoot_back.velocity<0)
                {
                        Shoot_back.velocity=0;
                }
                
                if (Shoot_front.velocity>ERUPT_SHOOT_SPEED_FRONT)//设定前轮速度上下限
                {
                        Shoot_front.velocity=ERUPT_SHOOT_SPEED_FRONT;
                }else if (Shoot_front.velocity<0)
                {
                        Shoot_front.velocity=0;
                }



                Gimbal.location_pitch+=(Channel_1-1024)*Gain_pitch;
                if(Gimbal.location_pitch<DM4310_pitch.location_min){
                        Gimbal.location_pitch=DM4310_pitch.location_min;
                }else if(Gimbal.location_pitch>DM4310_pitch.location_max){
                        Gimbal.location_pitch=DM4310_pitch.location_max;
                }

        }

//以下将对右面开关进行分析，右面的开关主要是负责射击机构的状态
        if(State_right==Remote_Switch_Right3&&State_right_last==Remote_Switch_Right2)
        {
                // //非发射状态
                // //在右侧档位为2和3时，若后摩擦轮开启（检测一组即可）gimbal_auto设为1，然后设置后摩擦轮关闭，反之亦然
                // //该函数并不会拨弹，不太清楚意义何在，先注释掉看看
                // if(Shoot_back.state_friction==0){
                //         Gimbal.gimbal_auto=1;
                //         Shoot_back.state_friction=1;
                // }else{
                //         Shoot_back.state_friction=0;
                //         Gimbal.gimbal_auto=0;
                // }
                
        }
        if(State_right==Remote_Switch_Right1&&Shoot_back.state_friction==1&&Shoot_back.plate_locked==0)
        {
                //当右侧档位为1时，进入发射状态
                Remote.State_right1_num++;//开始计时
                if(State_right_last==Remote_Switch_Right2)//
                {
                        Shoot_back.plate_location+=60.0f;
                        Shoot_back.state_plate=1;
                }
                if(Remote.State_right1_num>142)
                {
                        Shoot_back.plate_location+=1.2f;
                        Shoot_back.state_plate=2;
                }

        }else{
               Remote.State_right1_num=0; 
        }
}

void Class_Remote_data::remote_keyboard_control(){//用键盘控制机器人的函数
//        Bounce_speed_planning();//弹速规划

	if(Key_Ctrl == 0 && Key_ctrl_last == 1)
	{
		// if(Gyro_state ==0)
		// {
		// 	Gyro_state = 1;	//开小陀螺
		// }
		// else
		// {
			Gyro_state = 0;//关小陀螺
                        Chassis.velocity_angle=0;   
                        gyro_state_flag_left = 0; 
                        gyro_state_flag_right= 0;
		// }
	}
	
	if(Key_Shift == 0)//关闭超级电容
	{
		Super_Cup.Power_Limit_Max = Referee.Game_robot_status.chassis_power_limit - 5;
	}
	else//开启超级电容
	{
		if(Super_Cup.get_Energy_rest() > 130.0f)
		{
			Super_Cup.Power_Limit_Max = Referee.Game_robot_status.chassis_power_limit - 5 + 50;
		}
		else
		{
			Super_Cup.Power_Limit_Max = Referee.Game_robot_status.chassis_power_limit - 5;
		}
	}
	
	
	if(Gyro_state == 0)
	{
		Chassis.state=Remote_Switch_Left1;
                if(Key_Q||Key_E)
                {
                        Gyro_state = 2;
                }
		
	} 
	else if(Gyro_state == 1)//小陀螺模式下的小陀螺
	{
		Chassis.state=state_gyro;
		if((Key_Q == 0)&&(Key_Q_last == 1))
                {
                        if(gyro_state_flag_right==0)
                        {
                                if(gyro_state_flag_left == 0)
                                {
                                        Chassis.velocity_angle-=Chassis_v_omega_max;     
                                        gyro_state_flag_left = 1;  
                                }
                                else
                                {
                                        Chassis.velocity_angle=0;     
                                        gyro_state_flag_left = 0;  
                                } 
                        } 
                        else
                        {
                                gyro_state_flag_right = 0;
                                gyro_state_flag_left  = 1;
                                Chassis.velocity_angle=0; 
                                Chassis.velocity_angle-=Chassis_v_omega_max;   
                        }            
                }
        
                if((Key_E == 0)&&(Key_E_last == 1))
                {
                        if(gyro_state_flag_left==0)
                        {
                                if(gyro_state_flag_right == 0 )
                                {
                                        Chassis.velocity_angle=Chassis_v_omega_max;
                                        gyro_state_flag_right = 1;
                                }
                                else
                                {
                                        Chassis.velocity_angle =0;       
                                        gyro_state_flag_right = 0;
                                }
                        }
                        else
                        {
                                gyro_state_flag_right = 1;
                                gyro_state_flag_left  = 0;
                                Chassis.velocity_angle=0; 
                                Chassis.velocity_angle=Chassis_v_omega_max;   
                        }               
                }
	}
        else //点按小陀螺模式
        {
                Chassis.state=state_gyro;
                if(Key_Q)
                {
                        Chassis.velocity_angle-=Chassis_v_omega_max;                
                }
                else
                {
                        Chassis.velocity_angle-=0;                 
                }
                if(Key_E)
                {
                        Chassis.velocity_angle=Chassis_v_omega_max;                
                }
                else
                {
                        Chassis.velocity_angle=0;                 
                }
        }

        if((Key_F == 0)&&(Key_F_last == 1))
        {
                Gyro_state = 1;
        }
        
        Gimbal.location_yaw+=mouse_x_value * MOUSE_GAIN_YAW;			//云台pitch轴和yaw轴的控制
        Gimbal.location_pitch+=mouse_y_value * MOUSE_GAIN_PITCH;
        //因为鼠标运动噪声太大，保留遥控器控制
        Gimbal.location_yaw+=(Channel_0-1024)*Gain_yaw;
        Gimbal.location_pitch+=(Channel_1-1024)*Gain_pitch;

       if(Gimbal.location_pitch<DM4310_pitch.location_min){
                Gimbal.location_pitch=DM4310_pitch.location_min;
        }else if(Gimbal.location_pitch>DM4310_pitch.location_max){
                Gimbal.location_pitch=DM4310_pitch.location_max;
        }

        if(Key_D){
                Chassis.velocity_x=Chassis_v_x_max;                
        }else{
                Chassis.velocity_x=0;                 
        }
        if(Key_A){
                Chassis.velocity_x-=Chassis_v_x_max;                
        }else{
                Chassis.velocity_x-=0;                 
        }


        
        if(Key_W){
                Chassis.velocity_y=Chassis_v_y_max;                
        }else{
                Chassis.velocity_y=0;                 
        }
        if(Key_S){
                Chassis.velocity_y-=Chassis_v_y_max;                
        }else{
                Chassis.velocity_y-=0;                 
        }

        if(Mouse_right)
                {  
                        Gimbal.gimbal_auto = 1;//开启自瞄
                }
                else
                {
                        Gimbal.gimbal_auto = 0;//关闭自瞄
                }
	   

		if(Mouse_left)
		{
			if(Referee.Game_robot_status.mains_power_shooter_output == 1)//判断发射机构是否上电
			{
                                Remote.Mouse_left_num++;
                                if(Mouse_left_last == 0)//点射
                                {

                                        Shoot_back.plate_location+=60.0f;
                                        Shoot_back.state_plate=1;
                                }
                                if(Remote.Mouse_left_num > 20)//连射
                                {
                                        Shoot_back.plate_location+=Firing_frequency;
                                        Shoot_back.state_plate=2;
                                }
                        }
		}
		else
		{
			Remote.Mouse_left_num = 0;
			// Remote.friction_num++;
			// if(friction_num > 142)
			// {
			// 	Shoot.state_friction=0;//关闭摩擦轮
			// 	Remote.friction_num = 0;
			// }
		}
}



