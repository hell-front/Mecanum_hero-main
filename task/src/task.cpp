#include "task.h"
#include "main.h"
#include "Ablock.h"
#include "moter.h"
#include "Chassis.h"
#include "shoot.h"
#include "gimbal.h"
#include "string.h"
#include "Filter.h"
#include "referee.h"
#include "remote.h"
#include "imu_mini.h"
#include "mini_pc.h"
#include "Pan_Tilt_AHRS.h"

rt_thread_t led_galloping;
rt_thread_t task_init;
rt_thread_t task_serialplot;
rt_thread_t mpu6500_communication;
rt_thread_t task_heating_imu;
//rt_thread_t task_chassis;
rt_thread_t task_gimbal;
rt_thread_t task_gimbal_init;
rt_thread_t task_shoot;
rt_thread_t task_remote_check;
rt_thread_t task_referee_ui;
rt_thread_t task_send_data_to_miniPC;
rt_thread_t task_test_motor;


struct rt_thread task_chassis;

rt_sem_t sem_imu_heated;
rt_sem_t sem_can_Tx_full;
rt_sem_t sem_mini_pc;


extern DM_motor  test_motor;


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_spi5_rx;
extern DMA_HandleTypeDef hdma_spi5_tx;
extern SPI_HandleTypeDef hspi5;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern ADC_HandleTypeDef hadc1;


/*----------在Chassis.c文件�?定义的变�?----------*/
extern Class_Chassis Chassis;
extern C620_driver C620_chassis_1;
extern C620_driver C620_chassis_2;
extern C620_driver C620_chassis_3;
extern C620_driver C620_chassis_4;
extern uint32_t adc_buf;

extern Class_Super_Cup Super_Cup;
/*----------在gimbal.c文件�?定义的变�?----------*/
extern Class_Gimbal Gimbal;
extern DM_motor DM4310_pitch;
extern DM_motor DM6006_yaw;
extern uint8_t auto_aim_buf[];


/*----------在shoot.c文件�?定义的变�?----------*/
extern Class_Shoot Shoot;
extern C620_driver C620_plate;
extern C620_driver friction_left_front;
extern C620_driver friction_right_front;
extern C620_driver friction_left_back;
extern C620_driver friction_right_back;


/*----------在referee.cpp文件�?定义的变�?----------*/
extern Referee_System Referee;
extern uint8_t referee_buf[];
extern graphic_data_struct_t UI_Target[7];
extern uint8_t UI_flag;

extern Class_Remote_data Remote;
extern int32_t Shoot_speed_flag;
extern int16_t test_flag;
extern int32_t Proport_energy_remaining;
extern float Firing_frequency;
extern Class_MPU6500 Mpu_6500;


extern Class_imu_mini Imu_mini;

extern Class_Mini_pc Data_with_miniPC;


static rt_uint8_t rt_chassis_stack[2048];//线程栈

extern uint8_t remote_buf[18];


void Task_Init(void *parameter){
    Referee.Referee_system_init(&huart6);//将huart6和referee绑定起来
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1,remote_buf,18);//初�?�化遥控器接受函�?
    HAL_UARTEx_ReceiveToIdle_IT(&huart3,auto_aim_buf,18);//miniPC给我回传的视觉的数据
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6,referee_buf,REFER_NUM_MAX);//初�?�化裁判系统接收函数
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7,Imu_mini.buf_receive,263);//初�?�化裁判系统接收函数
    HAL_UARTEx_ReceiveToIdle_IT(&huart8,Pan_Tilt_AHRS_Rx_Buff,Pan_Tilt_AHRS_RX_BUFFER_SIZE);

    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    can_filter_mask32_init(&hcan1,0,CAN_ID_STD,CAN_FILTER_FIFO0,0x201,0x7f0);
    can_filter_mask32_init(&hcan2,15,CAN_ID_STD,CAN_FILTER_FIFO0,0x201,0x7f0);
    can_filter_mask32_init(&hcan2,15,CAN_ID_STD,CAN_FILTER_FIFO0,0x201,0x700);
	can_filter_mask32_init(&hcan1,1,CAN_ID_STD,CAN_FILTER_FIFO0,0x201,0x000);

    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc_buf, 1);



    //MPU_6500_init();//陀螺仪初�?�化函数
    
    sem_imu_heated=rt_sem_create("IMU_Heated",1,RT_IPC_FLAG_FIFO);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
    sem_can_Tx_full=rt_sem_create("CAN_Mailbox_Full",3,RT_IPC_FLAG_FIFO);//建立信号量，防�??can发送邮箱溢�?
    sem_mini_pc=rt_sem_create("MINIPC_Full",1,RT_IPC_FLAG_FIFO);

    //task_heating_imu=rt_thread_create("IMU_heating",Task_heating_IMU,RT_NULL,512,9,1);
    task_serialplot=rt_thread_create("SerialPlot",Task_SerialPlot,RT_NULL,512,9,1);
    // rt_thread_startup(task_heating_imu);
    rt_thread_startup(task_serialplot);



    rt_sem_take(sem_imu_heated,15000);              //设置信号量造成阻�?�，�?保在陀螺仪稳定后再进�?�定标去静差
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
    
    //rt_enter_critical();//进入临界区，关闭任务调度，保证去�?MPU6500静态�??�?过程不受其他任务干扰
    //MPU6500_get_errorG();
    //rt_exit_critical();//离开临界区，开�?任务调度�?
    // HAL_TIM_Base_Start_IT(&htim13);//开�?角度计算的中�?，�?�中�?主�?�用来�?�算角度

    rt_thread_delay(50);

    // while(GM6020_yaw.PROCESSED==0){
        
    // }m
    rt_thread_delay(50);
    Chassis_Mecanum_wheel_init();
    Chassis_Mecanum_wheel_init();  //底盘初�?�化，主要是初�?�化底盘电机的pid，初始化速度�?0防�?�暴�?
    Shoot_init();                   //云台初�?�化，内容同�?
    Gimbal_init();                  //射击初�?�化，内容同�?



    led_galloping=rt_thread_create("LED",LED_Galloping,RT_NULL,512,10,1);//跑马�?，直观反映RTOS�?否运�?
    rt_thread_startup(led_galloping);

    task_remote_check=rt_thread_create("LED",Task_Remote_Check,RT_NULL,512,9,1);//跑马�?，直观反映RTOS�?否运�?
    rt_thread_startup(task_remote_check);

    //mpu6500_communication=rt_thread_create("MPU6500_read",MPU6500_Communication,RT_NULL,512,9,50);
	//rt_thread_startup(mpu6500_communication);


    task_gimbal_init=rt_thread_create("Gimbal_init",Task_Gimbal_init,RT_NULL,512,9,2);
    rt_thread_startup(task_gimbal_init);

    //task_chassis=rt_thread_create("Chassis",Task_Chassis_tristeer_wheel,RT_NULL,2048,10,2);
    //rt_thread_init(&task_chassis,"Chassis",Task_Chassis_tristeer_wheel,NULL,rt_chassis_stack,sizeof(rt_chassis_stack),10,2);
    task_gimbal=rt_thread_create("Gimbal",Task_Gimbal,RT_NULL,2048,10,2);
    task_shoot=rt_thread_create("Shoot",Task_Shoot,RT_NULL,2048,10,2);
	//task_referee_ui = rt_thread_create("UI",Task_Referee_UI,RT_NULL,2048,10,2);
    //rt_thread_startup(&task_chassis);
    rt_thread_startup(task_gimbal);
    rt_thread_startup(task_shoot);
	// rt_thread_startup(task_referee_ui);

    // task_send_data_to_miniPC=rt_thread_create("send_to_minipc",Task_Send_data_to_miniPC,RT_NULL,2048,10,2);
    // rt_thread_startup(task_send_data_to_miniPC);

    // task_test_motor=rt_thread_create("Test_motor",Task_test_motor,RT_NULL,2048,10,2);
    //rt_thread_startup(task_test_motor);

    rt_thread_delete(task_init);

}

void Task_test_motor(void *parameter)
{
    float i;
    rt_thread_delay(1000);
     if (DM4310_pitch.state>=8)
    {
        Send_Data_DM_Control(&hcan2,0x210,3,2);//清除错误信号
        rt_thread_delay(200);//发出控制信息之后需要一定的处理时间
    }
        
    Send_Data_DM_Control(&hcan2,0x210,1,2);   
    rt_thread_delay(200);//发出控制信息之后需要一定的处理时间

    while(1)
    {
        for (i=0.0f;i<=2.0f;i=i+0.2f)
        {
            Send_Data_DM_V(&hcan2,0x210,i);
            rt_thread_delay(20);
        }
        for (i=2.0f;i>=0.0f;i=i-0.2f)
        {
            Send_Data_DM_V(&hcan2,0x210,i);
            rt_thread_delay(20);
        }
        for (i=0.0f;i>=-2.0f;i=i-0.2f)
        {
            Send_Data_DM_V(&hcan2,0x210,i);
            rt_thread_delay(20);
        }
        for (i=-2.0f;i<=0.0f;i=i+0.2f)
        {
            Send_Data_DM_V(&hcan2,0x210,i);
            rt_thread_delay(20);
        }




        // for (i=30.0f;i<=84.0f;i=i+0.2f)
        // {
        //     Send_Data_DM_PV(&hcan2,0x210,i,3.0f);
        //     rt_thread_delay(2);
        // }
        //  for (i=84.0f;i>=30.0f;i=i-0.2f)
        // {
        //     Send_Data_DM_PV(&hcan2,0x210,i,3.0f);
        //     rt_thread_delay(2);
        // }
        // Send_Data_DM_Control(&hcan2,0x210,1,1);   
        // rt_thread_delay(200);//发出控制信息之后需要一定的处理时间
        // Send_Data_DM_PV(&hcan2,0x210,80.0f,2.0f);
        // rt_thread_delay(300);


        // Shoot_resolution();//发射装置测试程序
        // Shoot_PID();
        // rt_sem_take(sem_can_Tx_full,0x01);
        // Send_Data_Dj(&hcan2,0x200,0,C620_plate.current_target,friction_right_front.current_target,friction_left_front.current_target);
        // rt_thread_delay(1);
        // Send_Data_Dj(&hcan2,0x1ff,0,0,friction_right_back.current_target,friction_left_back.current_target);
        // rt_thread_delay(1);

        // C620_plate.CAN_update=0;    
        // friction_left_front.CAN_update=0;
        // friction_right_front.CAN_update=0;
        // friction_left_back.CAN_update=0;
        // friction_right_back.CAN_update=0;

    }
}


void Task_heating_IMU(void *parameter){
        uint16_t PWM_heat;
        static uint32_t number=0;
        rt_sem_take(sem_imu_heated,0xff);
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);
        while (1)
        {   MPU_6500_Read_Temperature();
            PWM_heat=Limit_fuction(Mpu_6500.temperature_PID.PID_absolute(Mpu_6500.temperature_target,Mpu_6500.temperature_real),1000);
            __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,PWM_heat);
            if(Mpu_6500.temperature_real<46.0f&&Mpu_6500.temperature_real>44.0f){
                number++;

            }else{
                number=0;
            }
            if(number==100){
                rt_sem_release(sem_imu_heated);
                HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_SET);
            }
            rt_thread_delay(10);
        }
        
}


void Task_Chassis_tristeer_wheel(void *parameter){
    while (1)
    {
        if(Remote.connected&&C620_chassis_1.CAN_update&&C620_chassis_2.CAN_update&&C620_chassis_3.CAN_update&&C620_chassis_4.CAN_update){
            
            Chassis.Chassis_zeroing_if_OK();
            Chassis_Power_Limit();            
			//Chassis.Chassis_Movement_Plan();//该行代码已经归入到resolution里面
            Chassis_Mecanum_wheel_resolution();
            Chassis_Mecanum_wheel_PID();
            rt_sem_take(sem_can_Tx_full,0x01);
            Send_Data_Dj(&hcan1,0x200,C620_chassis_1.current_target,C620_chassis_2.current_target,C620_chassis_3.current_target,C620_chassis_4.current_target);





            rt_sem_take(sem_can_Tx_full,0x01);
            Super_Cup.Send_PowerLimt(Referee.Game_robot_status.chassis_power_limit);//向超级电容发送数据,根据当前底盘的功率限制来确定功率Referee.Game_robot_status.chassis_power_limit
            C620_chassis_1.CAN_update=0;
            C620_chassis_2.CAN_update=0;
            C620_chassis_3.CAN_update=0;
            C620_chassis_4.CAN_update=0;
        }else{
            rt_sem_take(sem_can_Tx_full,0x01);
            Send_Data_Dj(&hcan1,0x200,0,0,0,0);
            rt_sem_take(sem_can_Tx_full,0x01);
            Send_Data_Dj(&hcan1,0x1ff,0,0,0,0);            
        }
        rt_thread_delay(Chassis_Tick);

    }
    


}

void Task_Gimbal(void *parameter){

    Class_PID Angle_PID;
    float Angle_Zero;
    float Angle_Difference;

    while (1)
    {   
        // if(Remote.connected&&(DM4310_pitch.CAN_update||DM6006_yaw.CAN_update)){

            // Gimbal_resloution();
            // Gimbal_PID();
            // rt_sem_take(sem_can_Tx_full,0x01);


            Angle_PID.PID_init(0.98,0.01,0.05);
            Angle_Zero = Gimbal.location_pitch-2.2f;
            Angle_Difference = Angle_PID.PID_absolute(Gimbal.location_pitch,DM4310_pitch.location_real);
            DM4310_pitch.location_target = Angle_Zero + Angle_Difference;
            // if (DM4310_pitch.state>=8)
            // {
            //     Send_Data_DM_Control(&hcan2,0x210,3,2);//清除错误信号
            //     rt_thread_delay(1);//发出控制信息之后需要一定的处理时间
            // }
            Send_Data_DM_Mit(&hcan2,0x206,DM4310_pitch.location_target,4.0f,70.0f,0.5f,2.0f);
            rt_thread_delay(3);
            // Send_Data_DM_PV(&hcan2,0x206,Gimbal.location_pitch,2.0f);
            // rt_thread_delay(2);
            // Send_Data_DM_V(&hcan2,0x206,DM4310_pitch.velocity_target);
            // Send_Data_DM_V(&hcan2,0x205,DM6006_yaw.velocity_target);


            DM4310_pitch.CAN_update=0;
            DM6006_yaw.CAN_update=0;
        // }else{
        //     rt_sem_take(sem_can_Tx_full,0x01);
        //     Send_Data_Dj(&hcan2,0x1ff,0,0,0,0);            
        // }

        rt_thread_delay(Gimbal_Tick);
    }
    

}

void Task_Gimbal_init(void *parameter){
    Gimbal.Delta_Yaw_Max=0.045f*Gimbal_Tick;
    Gimbal.Delta_Pitch_Max=0.045f*Gimbal_Tick;
    uint32_t gimbal_init_time=0;
    Gimbal.Init_OK=0;
    rt_thread_delay(500);
    if (DM4310_pitch.state>=8)
        {
            Send_Data_DM_Control(&hcan2,0x205,3,0);//清除错误信号
            rt_thread_delay(1);//发出控制信息之后需要一定的处理时间
        }
    rt_thread_delay(100);
    Send_Data_DM_Control(&hcan2,0x206,1,0);
    rt_thread_delay(100);
    Send_Data_DM_Control(&hcan2,0x205,1,0);    
    rt_thread_delay(100);//发出控制信息之后需要一定的处理时间
    while(gimbal_init_time<450){
        gimbal_init_time++;
        rt_thread_delay(10);
    }
    Gimbal.Init_OK=1;
    rt_thread_delete(task_gimbal_init);

}

void Task_Shoot(void *parameter){
    while (1)
    {
        // if(Remote.connected&&friction_left_front.CAN_update&&friction_right_front.CAN_update&&friction_left_back.CAN_update&&friction_right_back.CAN_update){

            

        //     Shoot_resolution();
        //     Shoot_PID();
        //     rt_sem_take(sem_can_Tx_full,0x01);
        //     Send_Data_Dj(&hcan2,0x200,0,C620_plate.current_target,friction_right_front.current_target,friction_left_front.current_target);
        //     Send_Data_Dj(&hcan2,0x1ff,0,0,friction_right_back.current_target,friction_left_back.current_target);
        

            

        //     C620_plate.CAN_update=0;
        //     friction_left_front.CAN_update=0;
        //     friction_right_front.CAN_update=0;
        //     friction_left_back.CAN_update=0;
        //     friction_right_back.CAN_update=0;

        // }else{
        //     rt_sem_take(sem_can_Tx_full,0x01);
        //     Send_Data_Dj(&hcan2,0x200,0,0,0,0);  
        // }
        
        // rt_thread_delay(Shoot_Tick);.6
        Shoot_resolution();//发射装置测试程序
        Shoot_PID();
        rt_sem_take(sem_can_Tx_full,0x01);
        Send_Data_Dj(&hcan2,0x200,0,C620_plate.current_target,friction_right_front.current_target,friction_left_front.current_target);
        rt_thread_delay(1);
        Send_Data_Dj(&hcan2,0x1ff,0,0,friction_right_back.current_target,friction_left_back.current_target);
        rt_thread_delay(1);

        C620_plate.CAN_update=0;    
        friction_left_front.CAN_update=0;
        friction_right_front.CAN_update=0;
        friction_left_back.CAN_update=0;
        friction_right_back.CAN_update=0;

    }
    


}


void Task_Referee(void *parameter){
    while(1){

    }
}





extern float aaa;
void Task_SerialPlot(void *parameter){
    uint8_t  rx_buf[37];
    
    float temp1;
    float temp2;
    float temp3;
    float temp4;
    float temp5;
    float temp6;
    float temp7;
    float temp8;
    float temp9;
    rx_buf[0]=0xAB;

        while (1){

            //temp1 = C620_plate.get_current_real();
//            temp2 = Referee.Power_heat_data.chassis_power;
			// temp1=Imu_mini.AHRSdata_Packet.Roll;
			// temp2 =Imu_mini.AHRSdata_Packet.Pitch;
			// temp3 =Imu_mini.AHRSdata_Packet.Yaw;
			// temp4 =Imu_mini.AHRSdata_Packet.YawSpeed;
			// temp5 = Super_Cup.get_Chassis_power_real();
            //  temp1 = GM6020_yaw.get_location_real();
            //  temp2 = GM6020_pitch.get_location_real();


            // temp1 = Gimbal.Delta_pitch;
            // temp2 = Gimbal.Predicted_Delta_Pitch;
            // temp3 = Gimbal.pitch_comp;
            // temp4 = Gimbal.Delta_Yaw;
            // temp4=Referee.Power_heat_data.chassis_power;
            // temp5=Chassis.K_vel*Chassis.velocity_y;
            // temp6=Chassis.get_velocity_y_planned();
            // temp6=0.001f*0.000146322566f*GM6020_chassis_1.get_velocity_real()*GM6020_chassis_1.get_current_real();
            
//            temp1 = GM6020_yaw.location_target;
//            temp2 = GM6020_yaw.get_location_real();
//            temp3 = GM6020_yaw.velocity_target;
//            temp4 = -Pan_Tilt_AHRS.GetPitchOmega()  - Imu_mini.Yaw_speed_real * 180 / PI;
//            temp5 = GM6020_yaw.current_target;
//            temp6 = GM6020_yaw.get_current_real();
//            temp7=Chassis.zeroing_state;
//            temp8=Chassis.angle_target;
//            temp9 = -Imu_mini.Angle_Yaw_real-Chassis.angle_init;
            temp1 = Gimbal.location_pitch;
            temp2 = DM4310_pitch.location_real;
            temp3 = DM4310_pitch.location_target;
            // temp1 = friction_left_back.velocity_real;//测试摩擦轮
            // temp2 = friction_left_front.velocity_real;
            // temp3 = friction_right_back.velocity_real;
            // temp4 = friction_right_front.velocity_real;
            memcpy(rx_buf+1,&temp1,4);
            memcpy(rx_buf+5,&temp2,4);
            memcpy(rx_buf+9,&temp3,4);
            memcpy(rx_buf+13,&temp4,4);
            memcpy(rx_buf+17, &temp5, 4);
			memcpy(rx_buf+21, &temp6, 4);
            memcpy(rx_buf+25, &temp7, 4);
			memcpy(rx_buf+29, &temp8, 4);
            memcpy(rx_buf+33, &temp9, 4);
            HAL_UART_Transmit_DMA(&huart2,rx_buf,37);
            rt_thread_delay(10);
        }
        
}

void LED_Galloping(void *parameter){
    
    while(1){
        Galloping();
        rt_thread_delay(LED_INTERVAL); 
    }

}

void MPU6500_Communication(void *parameter){

        
        static rt_tick_t time;
        
        while (1)
        {
           MPU_6500_Read_DMA();
           rt_thread_delay_until(&time,IMU_Time);
           
        }
        
}

void Task_Remote_Check(void *parameter){//检查遥控器是否连接

    while(1){
        if(Remote.update){
            Remote.connected=1;//表示这时已经连接上
            Remote.update=0;
        }else{
            Remote.connected=0;//表示这时候未连接上
        }
        rt_thread_delay(50);
    }

}

void Task_Referee_UI(void *parameter)//发送自定义UI
{
    if(UI_flag == 1)
        {
            Draw_Target();
            rt_thread_delay(200);
            Draw_auto_shoot();
            rt_thread_delay(200);
            Draw_gyro();
            rt_thread_delay(200);
            Draw_shoot_level();
            rt_thread_delay(200);
            Draw_super_cup();
            rt_thread_delay(200);
            Draw_parking_line();
            rt_thread_delay(200);
            UI_flag = 0;
        }
        while(1)
        {
            Draw_dynamic_icon();
            rt_thread_delay(110);
            Draw_dynamic_chassis();
            rt_thread_delay(110);
        }

}

void Task_Send_data_to_miniPC(void *parameter)
{

    while(1){
        if(Data_with_miniPC.process==0){
            Data_with_miniPC.Data_processing_to_minipc();
        }else{
            rt_thread_delete(task_send_data_to_miniPC);
        }
        
        rt_thread_delay(10);
    }
}
