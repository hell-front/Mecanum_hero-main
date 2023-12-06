#include "main.h"
#include "Chassis.h"
#include "Ablock.h"
#include "remote.h"
#include "stdlib.h"
#include "moter.h"
#include "math.h"
#include "gimbal.h"
#include "imu_mini.h"




extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern ADC_HandleTypeDef hadc1;



C620_driver C620_chassis_1(0x01,19.0f);//右前
C620_driver C620_chassis_2(0x02,19.0f);//左前
C620_driver C620_chassis_3(0x03,19.0f);//左后
C620_driver C620_chassis_4(0x04,19.0f);//右后


//struct  Chassis_move Chassis={0,0,0,1};
Class_Chassis Chassis;
Class_Super_Cup Super_Cup(&hcan1,0x210,50);

uint32_t adc_buf;

extern Class_Gimbal Gimbal;


//extern float YAW;
extern Class_Remote_data Remote;
//extern Class_MPU6500 Mpu_6500;
extern Class_imu_mini Imu_mini;

Class_Chassis::Class_Chassis(){
    velocity_angle=0;
    velocity_angle_planned=0;
    velocity_x=0;
    velocity_x_planned=0;
    velocity_y=0;
    velocity_y_planned=0;
    zeroing_state=0;
    state=1;
    K_vel=1;

}



void Class_Chassis::Chassis_init(float DELTA_VEL_X_PLUS,float DELTA_VEL_Y_PLUS,float DELTA_VEL_ANGLE_PLUS,float DELTA_VEL_X_MINUS,float DELTA_VEL_Y_MINUS,float DELTA_VEL_ANGLE_MINUS)
{
    Delta_vel_x_plus=DELTA_VEL_X_PLUS;
    Delta_vel_y_plus=DELTA_VEL_Y_PLUS;
    Delta_vel_angle_plus=DELTA_VEL_ANGLE_PLUS;

    Delta_vel_x_minus=DELTA_VEL_X_MINUS;
    Delta_vel_y_minus=DELTA_VEL_Y_MINUS;
    Delta_vel_angle_minus=DELTA_VEL_ANGLE_MINUS;

    Chassis.state=state_normal;

    Power_PID.PID_init(200, 0.1, 0);
    Power_PID.PID_anti_integ_saturated_init(Current_MAX_M3508,-Current_MAX_M3508);



}
//对底盘运动的速度进行规划，让每次速度的增加或者减少的幅度在一个范围内
void Class_Chassis::Chassis_Movement_Plan(){
    velocity_plan_c(K_vel*velocity_x,&velocity_x_planned,Delta_vel_x_plus,Delta_vel_x_minus);
    velocity_plan_c(K_vel*velocity_y,&velocity_y_planned,Delta_vel_y_plus,Delta_vel_y_minus);
    velocity_plan_c(K_vel*velocity_angle,&velocity_angle_planned,Delta_vel_angle_plus,Delta_vel_angle_minus);   


}


void Chassis_Power_Limit(){
    
    float Power_3508;//表示扣除6020后3508的剩余功率
    float Curr_3508;//表示3508的最大电流

    float chassis_power = (adc_buf * 3.3f / 4096.0f - 1.65f) * 120;
    
    if(chassis_power<Super_Cup.Power_Limit_Max){//当能量比较充足的时候
        Chassis.K_vel+=0.01f;
    }else{//当底盘的能量不怎么充足的时候或者底盘功率达到上限时
        Chassis.K_vel-=0.015f;
    }
    if(Chassis.K_vel>1.0f){
        Chassis.K_vel=1.0f;
    }
    if(Chassis.K_vel<0){
        Chassis.K_vel=0;
    }
    //GM6020返回电流和实际电流（A）的关系为：0.000146322566f
    //6020输出电压值和实际电压值（A）的关系为：



    Power_3508=Super_Cup.Power_Limit_Max;

    Curr_3508=Chassis.Power_PID.PID_anti_integral_saturated(Power_3508,chassis_power);//计算出来驱动电机电流的最大值

    if(Curr_3508>Current_MAX_M3508){
        Curr_3508=Current_MAX_M3508;
    }
    if (Curr_3508<0)
    {
        Curr_3508=0;
    }
    


    C620_chassis_1.velocity_PID.PID_anti_integ_saturated_init(Curr_3508,-Curr_3508);
    C620_chassis_2.velocity_PID.PID_anti_integ_saturated_init(Curr_3508,-Curr_3508);
    C620_chassis_3.velocity_PID.PID_anti_integ_saturated_init(Curr_3508,-Curr_3508);
    C620_chassis_4.velocity_PID.PID_anti_integ_saturated_init(Curr_3508,-Curr_3508);
}

void Class_Chassis::Chassis_zeroing_if_OK(){//判定此时是否需要强制修改角速度以求回到归0模式，方法为强制修改角速度
        if(Chassis.state==state_normal){


            if(Chassis.zeroing_state){//判定底盘此刻是否需要归零，如果需要，则执行该步骤操作，该步骤操作主要是修改角速度
                Chassis.velocity_angle=Chassis.PID_angle.PID_anti_integral_saturated(Chassis.angle_target,-Imu_mini.Angle_Yaw_real-Chassis.angle_init);
                // Chassis.velocity_angle=10 * (Chassis.angle_target-(-Imu_mini.Angle_Yaw_real-Chassis.angle_init));

            } 
        }
}






void Chassis_Mecanum_wheel_init(){




    Chassis.Chassis_init(3840*0.001*Chassis_Tick,3840*0.001*Chassis_Tick,5760*0.001*Chassis_Tick,9600*0.001*Chassis_Tick,9600*0.001*Chassis_Tick,9600*0.001*Chassis_Tick);
    Chassis.PID_angle.PID_init(50,0,0);
    Chassis.PID_angle.PID_anti_integ_saturated_init(10000.0f,-10000.0f);
    
    C620_chassis_1.velocity_PID.PID_init(12.0f,0.03f*Chassis_Tick,0);
    C620_chassis_1.velocity_PID.PID_anti_integ_saturated_init(16384,-16384);
    
    C620_chassis_2.velocity_PID.PID_init(12.0f,0.03f*Chassis_Tick,0);
    C620_chassis_2.velocity_PID.PID_anti_integ_saturated_init(16384,-16384);
    
    C620_chassis_3.velocity_PID.PID_init(12.0f,0.03f*Chassis_Tick,0);
    C620_chassis_3.velocity_PID.PID_anti_integ_saturated_init(16384,-16384);

    C620_chassis_4.velocity_PID.PID_init(12.0f,0.03f*Chassis_Tick,0);
    C620_chassis_4.velocity_PID.PID_anti_integ_saturated_init(16384,-16384);

    






}


void Chassis_Mecanum_wheel_resolution(){


        //Chassis.Chassis_Movement_Plan();

        float v_x=0,v_y=0;
        float Theta=0;          //表示车旋�???的�?�度，为角度制表�???
//        float Theta_ackerman=0;//表示三轮车模式下前轮旋转的�?�度，�?��?�度最大为±60°


            
        //if(Remote.State_1==state_normal){
        if(Chassis.state==state_normal){

            Chassis.Chassis_Movement_Plan();


            Theta=-PI*Gimbal.yaw_real/180.0f;//将角度值化为弧度制
            
            v_x=-Chassis.get_velocity_x_planned();
            v_y=Chassis.get_velocity_y_planned();          


            //v_x=-Chassis.get_velocity_x_planned()*cosf(Theta)+Chassis.get_velocity_y_planned()*sinf(Theta);
            //v_y=Chassis.get_velocity_x_planned()*sinf(Theta)+Chassis.get_velocity_y_planned()*cosf(Theta);

//底盘处于云台跟随模式下，这个时候定义的正方向为云台的方向
            C620_chassis_1.velocity_target=v_y-v_x+Chassis.get_velocity_angle_planned()*(wheel_length+wheel_width);
            C620_chassis_2.velocity_target=v_x+v_y+Chassis.get_velocity_angle_planned()*(wheel_length+wheel_width);
            C620_chassis_3.velocity_target=-(v_y-v_x-Chassis.get_velocity_angle_planned()*(wheel_length+wheel_width));
            C620_chassis_4.velocity_target=-(v_y+v_x-Chassis.get_velocity_angle_planned()*(wheel_length+wheel_width));







            //aaa=angle_target1;
            

 
                
                        
        //}else if (Remote.State_1==state_gyro){
        }else if (Chassis.state==state_gyro){
            Chassis.zeroing_state=0;//如果突然进入的小陀螺模式，则默认归零

            Chassis.Chassis_Movement_Plan();

            Theta=-PI*Gimbal.yaw_real/180.0f;//将角度值化为弧度制
            
            


            v_x=-Chassis.get_velocity_x_planned()*cosf(Theta)+Chassis.get_velocity_y_planned()*sinf(Theta);
            v_y=Chassis.get_velocity_x_planned()*sinf(Theta)+Chassis.get_velocity_y_planned()*cosf(Theta);


            float center_theta = 30.0f / 180.0f * PI; //小陀螺模式下中心-顶点连线相对于底边的角度
            //缓解高速运动时云台抖动的问题
            C620_chassis_1.velocity_target=v_y-v_x+Chassis.get_velocity_angle_planned()*(wheel_length+wheel_width);
            C620_chassis_2.velocity_target=v_x+v_y+Chassis.get_velocity_angle_planned()*(wheel_length+wheel_width);
            C620_chassis_3.velocity_target=-(v_y-v_x-Chassis.get_velocity_angle_planned()*(wheel_length+wheel_width));
            C620_chassis_4.velocity_target=-(v_y+v_x-Chassis.get_velocity_angle_planned()*(wheel_length+wheel_width));




   

        }



            


          
}



void Chassis_Mecanum_wheel_PID(){


    C620_chassis_1.current_target=C620_chassis_1.velocity_PID.PID_anti_integral_saturated(C620_chassis_1.velocity_target,C620_chassis_1.get_velocity_real());
    C620_chassis_2.current_target=C620_chassis_2.velocity_PID.PID_anti_integral_saturated(C620_chassis_2.velocity_target,C620_chassis_2.get_velocity_real());
    C620_chassis_3.current_target=C620_chassis_3.velocity_PID.PID_anti_integral_saturated(C620_chassis_3.velocity_target,C620_chassis_3.get_velocity_real());
    C620_chassis_4.current_target=C620_chassis_4.velocity_PID.PID_anti_integral_saturated(C620_chassis_4.velocity_target,C620_chassis_4.get_velocity_real());
    

}










void velocity_plan_c(float velocity,float *velocity_plan,float Delta_plus,float Delta_minus)//规划最大最小速度
{

    if(((velocity)*(*velocity_plan)>=0)&&((velocity)!=0)){
        if(fabsf(velocity)-fabsf(*velocity_plan)>=0){
            if(fabsf(velocity)-fabsf(*velocity_plan)<=Delta_plus){
                *velocity_plan=velocity;
            }else{
                *velocity_plan+=(velocity>0)?Delta_plus:(-Delta_plus);
            }
        }
        else{
            if(fabsf(*velocity_plan)-fabsf(velocity)<=Delta_plus){
                *velocity_plan=velocity;
            }else{
                *velocity_plan-=(velocity>0)?Delta_plus:(-Delta_plus);
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

void location_plan_c(float location,float *location_plan,float Delta_location)
{
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







void Class_Super_Cup::Send_PowerLimt(int16_t Power_l){
            CAN_TxHeaderTypeDef TX_header;
            Power_Limit=Power_l;
            if(Power_Limit<0){
                Power_Limit=0;
            }else if(Power_Limit>Power_Limit_Max){
                Power_Limit=Power_Limit_Max;
            }
            uint8_t Txbuf[8];
            uint32_t TxMAailBox;
          
            TX_header.DLC=8;

            TX_header.StdId=0x220;
            TX_header.IDE=CAN_ID_STD;
            TX_header.RTR=CAN_RTR_DATA;


            Txbuf[1]=(Power_Limit);
            Txbuf[0]=Power_Limit>>8;
            Txbuf[3]=0;
            Txbuf[2]=0;
            Txbuf[5]=0;
            Txbuf[4]=0;
            Txbuf[7]=0;
            Txbuf[6]=0;

            HAL_CAN_AddTxMessage(hcanp,&TX_header,Txbuf,&TxMAailBox);   
}




void Class_Super_Cup::Send_PowerLimt(){
            CAN_TxHeaderTypeDef TX_header;
            if(Power_Limit<0){
                Power_Limit=0;
            }else if(Power_Limit>Power_Limit_Max){
                Power_Limit=Power_Limit_Max;
            }
            uint8_t Txbuf[8];
            uint32_t TxMAailBox;
          
            TX_header.DLC=8;

            TX_header.StdId=0x220;
            TX_header.IDE=CAN_ID_STD;
            TX_header.RTR=CAN_RTR_DATA;


            Txbuf[1]=(Power_Limit);
            Txbuf[0]=Power_Limit>>8;
            Txbuf[3]=0;
            Txbuf[2]=0;
            Txbuf[5]=0;
            Txbuf[4]=0;
            Txbuf[7]=0;
            Txbuf[6]=0;

            HAL_CAN_AddTxMessage(hcanp,&TX_header,Txbuf,&TxMAailBox);   
}
/*
对应通信协议代码如下
get chassis adc(temp0,sbridge data) ;
data[2]=(int)bridge data.chassis power*100>>8;
data[l3]=bridge data.chassis power*100:
data[0]=(int) (0.5*8,75*bridge data,cap voltage*bridge data,cap voltage)>>8;
datall]=(int)(0.5*875bridge datacap voltagekbridge data.cap voltage):*/
void Class_Super_Cup::Can_Data_processing(uint8_t buf[])
{
    int16_t TEMP1;
    int16_t TEMP2;
    TEMP1=(int16_t)((buf[0]<<8)|buf[1]);
    TEMP2=(int16_t)((buf[2]<<8)|buf[3]);
    Energy_rest=TEMP1;
    Chassis_Power_real=TEMP2*0.01f;


    

}

