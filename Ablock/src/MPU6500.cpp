#include "MPU6500.h"
#include "main.h"
#include "stdio.h"
#include "math.h"





extern SPI_HandleTypeDef hspi5;



uint8_t MPU_6500_rx_buf[14];


Class_MPU6500 Mpu_6500;

void MPU_6500_init(){//设置陀螺仪初始化，包括硬件的初始化，和后续处理的初始化


		Mpu_6500.MPU6500_config();//陀螺仪相关处理参数的初始化，包括滤波器参数设置，温度PID参数设置

        uint8_t MPU6500_Init_Data[8][2] = {{ MPU6500_PWR_MGMT_1, 0x80 },     /* 重置陀螺仪*/ 
		        							{ MPU6500_PWR_MGMT_1, 0x03 },     /* 陀螺仪的时钟源设置 */ 
				        					{ MPU6500_PWR_MGMT_2, 0x00 },     /* 启动低通滤波器 Acc & Gyro */ 
						        			{ MPU6500_CONFIG, 0x04 },         /* 低通滤波器的频率41Hz */ 
								        	{ MPU6500_GYRO_CONFIG, 0x18 },    /* 设置角速度计的量程+-2000dps */ 
									        { MPU6500_ACCEL_CONFIG, 0x10 },   /* 设置加速度的量程+-8G */ 
									        { MPU6500_ACCEL_CONFIG_2, 0x02 }, /* 使能低�?�滤�???�????  设置 Acc 低�?�滤�???1?7?? */ 
									        { MPU6500_USER_CTRL, 0x20 },};    /* 使能 AUX */ 
	    for (uint8_t i = 0; i < 8; i++){

            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_RESET);
    		HAL_SPI_Transmit(&hspi5,MPU6500_Init_Data[i],2,0xff);//依次将陀螺仪的设置写入到陀螺仪的寄存器里
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_SET);
		    HAL_Delay(1);
        
        }
		

}



void MPU6500_get_errorG(){//获得陀螺仪的零票，该参数要到陀螺仪温度达到指定温度才可以获取
	int16_t errorGx[Error_number];
	int16_t errorGy[Error_number];
	int16_t errorGz[Error_number];
	int16_t Tempera[Error_number];
	int32_t sumx=0;
	int32_t sumy=0;
	int32_t sumz=0;
	int32_t sumt=0;


	for(uint8_t i=0;i<Error_number;i++){
		MPU_6500_Read_gyro();//读取陀螺仪的温度，取平均作为陀螺仪的预定温度，将温漂的影响降到最小
		Tempera[i]=(int16_t)((MPU_6500_rx_buf[0]<<8)|MPU_6500_rx_buf[1]);
		errorGx[i]=(int16_t)((MPU_6500_rx_buf[2]<<8)|MPU_6500_rx_buf[3]);
		errorGy[i]=(int16_t)((MPU_6500_rx_buf[4]<<8)|MPU_6500_rx_buf[5]);
		errorGz[i]=(int16_t)((MPU_6500_rx_buf[6]<<8)|MPU_6500_rx_buf[7]);
	
		sumx+=errorGx[i];
		sumy+=errorGy[i];
		sumz+=errorGz[i];
		sumt+=Tempera[i];
		//printf("%hd,%hd,%hd\n",errorGx[i],errorGy[i],errorGz[i]);
	}
	Mpu_6500.Gyro_x_error=-(sumx/(float)Error_number)*2000.0f/32768.0f;
	Mpu_6500.Gyro_y_error=-(sumy/(float)Error_number)*2000.0f/32768.0f;
	Mpu_6500.Gyro_z_error=-(sumz/(float)Error_number)*2000.0f/32768.0f;
	Mpu_6500.temperature_target=(sumt/(float)Error_number)/333.87f+21.0f;
//	printf("%f,%f,%f\n",Mpu_6500.Gyro_x_error,Mpu_6500.Gyro_y_error,Mpu_6500.Gyro_z_error);
	
	Mpu_6500.Q_init();//初始化陀螺仪的四元数


}


void MPU_6500_Read_Temperature(){//PID获取陀螺仪的温度，在陀螺仪SPI5通讯线程正常工作后，此函数不再运行
	float temp;
	uint8_t sip5_buf[1]={MPU6500_TEMP_OUT_H|0x80};
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_RESET);		
	HAL_SPI_Transmit(&hspi5, sip5_buf,1,0xff);
    HAL_SPI_Receive(&hspi5,MPU_6500_rx_buf,2,0xff);
	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_SET);
	temp=((int16_t)((MPU_6500_rx_buf[0]<<8)|(MPU_6500_rx_buf[1])))/333.87f+21.0f;
	Mpu_6500.set_temperature_measured(temp);
	Mpu_6500.temperature_real=Mpu_6500.Temp_K_Filter.kalman_Filter(temp);
//	printf("temp=%.4f\r\n",Mpu_6500.temperature_real);
}








void MPU_6500_Read_gyro(){//获取陀螺仪的角速度测量值

	    uint8_t sip5_buf[1]={0x41|0x80};					

		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_RESET);		
		HAL_SPI_Transmit(&hspi5, sip5_buf,1,0xff);
    	HAL_SPI_Receive(&hspi5,MPU_6500_rx_buf,8,0xff);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_SET);
		HAL_Delay(1);
}

void MPU_6500_Read_accel(){//获取陀螺仪的加速度测量值

	    uint8_t sip5_buf[1]={MPU6500_ACCEL_XOUT_H|0x80};					

		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_RESET);		
		HAL_SPI_Transmit(&hspi5, sip5_buf,1,0xff);
    	HAL_SPI_Receive(&hspi5,MPU_6500_rx_buf,6,0xff);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_SET);
		HAL_Delay(1);
}


void MPU_6500_Read_DMA(){//DMA获取陀螺仪的7个参数
	    uint8_t sip5_buf[1]={0x3B|0x80};


		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi5, sip5_buf,1,0xff);
    	HAL_SPI_Receive_DMA(&hspi5,MPU_6500_rx_buf,14);//在中SPI5回调函数中拉高片选引脚，结束SPI通讯
}

//陀螺仪相关处理参数的初始化，包括滤波器参数设置，温度PID参数设置
void Class_MPU6500::MPU6500_config(){
		Gyro_x_error=0;
		Gyro_y_error=0;
		Gyro_x_error=0;
		Angle_roll=0;
		Angle_yaw=0;
		Angle_pitch=0;
		Radian_pitch=0;
		Radian_roll=0;
		Radian_yaw=0;
		
		temperature_target=39;
		Gyro_x_radian_PID.PID_init(0.1,0.001f,0);
		Gyro_y_radian_PID.PID_init(0.1,0.001f,0);
		Gyro_z_radian_PID.PID_init(0.1,0.001f,0);
		
		temperature_PID.PID_init(80,0.1f,20);
		temperature_PID.PID_integ_separated_init(20);
		temperature_PID.PID_anti_integ_saturated_init(1000,0);
		temperature_PID.PID_differ_filter_init(0.25f);

		gyro_X_Filter.Kalman_filter_init(0,0.003,0.287);
		gyro_Y_Filter.Kalman_filter_init(0,0.003,0.287);
		gyro_Z_Filter.Kalman_filter_init(0,0.003,0.05);

		accel_X_Filter.Kalman_filter_init(0,0.003,0.0287);
		accel_Y_Filter.Kalman_filter_init(0,0.003,0.0287);
		accel_Z_Filter.Kalman_filter_init(0,0.003,0.0287);

		Temp_K_Filter.Kalman_filter_init(30.0f,0.003f,0.2f);
		
}




//由加速度计（假设陀螺仪初始时位于静止状态）获取陀螺仪的姿态角，假定初始偏航角为0度							
void Class_MPU6500::Q_init(){
	
	float sum_acc_x=0;
	float sum_acc_y=0;
	float sum_acc_z=0;

	for(uint8_t i=0;i<Error_number;i++){
		MPU_6500_Read_accel();
		Acceleration_x_measured=(int16_t)((MPU_6500_rx_buf[0]<<8)|MPU_6500_rx_buf[1]);
		Acceleration_y_measured=(int16_t)((MPU_6500_rx_buf[2]<<8)|MPU_6500_rx_buf[3]);
		Acceleration_z_measured=(int16_t)((MPU_6500_rx_buf[4]<<8)|MPU_6500_rx_buf[5]);
	
		sum_acc_x+=Mpu_6500.Acceleration_x_measured;
		sum_acc_y+=Mpu_6500.Acceleration_y_measured;
		sum_acc_z+=Mpu_6500.Acceleration_z_measured;
		//printf("%hd,%hd,%hd\n",errorGx[i],errorGy[i],errorGz[i]);
	}
	Gravity_x_measured=(sum_acc_x/(float)Error_number)*8.0f*9.8f/32768.0f;
	Gravity_y_measured=(sum_acc_y/(float)Error_number)*8.0f*9.8f/32768.0f;
	Gravity_z_measured=(sum_acc_z/(float)Error_number)*8.0f*9.8f/32768.0f;

	Gravity_measured=sqrtf(Gravity_x_measured*Gravity_x_measured+Gravity_y_measured*Gravity_y_measured+Gravity_z_measured*Gravity_z_measured);

	Radian_yaw=0;
	Radian_pitch=asinf(Gravity_y_measured/Gravity_measured);
	Radian_roll=-atan2f(Gravity_x_measured,Gravity_z_measured);
	

	
	//Q0=arm_cos_f32(Radian_yaw/2.0f)*arm_cos_f32(Radian_roll/2.0f)*arm_cos_f32(Radian_pitch/2.0f)+arm_sin_f32(Radian_yaw/2.0f)*arm_sin_f32(Radian_roll/2.0f)*arm_sin_f32(Radian_pitch/2.0f);
	//Q1=arm_cos_f32(Radian_yaw/2.0f)*arm_cos_f32(Radian_roll/2.0f)*arm_sin_f32(Radian_pitch/2.0f)+arm_sin_f32(Radian_yaw/2.0f)*arm_sin_f32(Radian_roll/2.0f)*arm_cos_f32(Radian_pitch/2.0f);
	//Q2=arm_cos_f32(Radian_yaw/2.0f)*arm_sin_f32(Radian_roll/2.0f)*arm_cos_f32(Radian_pitch/2.0f)-arm_sin_f32(Radian_yaw/2.0f)*arm_cos_f32(Radian_roll/2.0f)*arm_sin_f32(Radian_pitch/2.0f);
	//Q3=arm_sin_f32(Radian_yaw/2.0f)*arm_cos_f32(Radian_roll/2.0f)*arm_cos_f32(Radian_pitch/2.0f)-arm_cos_f32(Radian_yaw/2.0f)*arm_sin_f32(Radian_roll/2.0f)*arm_sin_f32(Radian_pitch/2.0f);

	Q0=cosf(Radian_yaw/2.0f)*cosf(Radian_roll/2.0f)*cosf(Radian_pitch/2.0f)-sinf(Radian_yaw/2.0f)*sinf(Radian_roll/2.0f)*sinf(Radian_pitch/2.0f);
	Q1=cosf(Radian_yaw/2.0f)*cosf(Radian_roll/2.0f)*sinf(Radian_pitch/2.0f)-sinf(Radian_yaw/2.0f)*sinf(Radian_roll/2.0f)*cosf(Radian_pitch/2.0f);
	Q2=cosf(Radian_yaw/2.0f)*sinf(Radian_roll/2.0f)*cosf(Radian_pitch/2.0f)+sinf(Radian_yaw/2.0f)*cosf(Radian_roll/2.0f)*sinf(Radian_pitch/2.0f);
	Q3=sinf(Radian_yaw/2.0f)*cosf(Radian_roll/2.0f)*cosf(Radian_pitch/2.0f)+cosf(Radian_yaw/2.0f)*sinf(Radian_roll/2.0f)*sinf(Radian_pitch/2.0f);

}

//更新陀螺仪的四元数，包括一阶龙哥库塔和四阶龙格库塔法，该函数在定时器中断函数中调用
void Class_MPU6500::Q_update(){
	float Q_mode;


	
	Gravity_real=sqrtf(Gravity_x_real*Gravity_x_real+Gravity_y_real*Gravity_y_real+Gravity_z_real*Gravity_z_real);
	if(Gravity_real!=0){//防止初始化导致分母为0导致程序崩掉
		Gravity_x_error=(Gravity_y_real*Gravity_z_guessed - Gravity_z_real*Gravity_y_guessed)/(9.8f*Gravity_real);
		Gravity_y_error=(Gravity_z_real*Gravity_x_guessed - Gravity_x_real*Gravity_z_guessed)/(9.8f*Gravity_real);
		Gravity_z_error=(Gravity_x_real*Gravity_y_guessed - Gravity_y_real*Gravity_x_guessed)/(9.8f*Gravity_real);

		Gyro_x_radian+=Gyro_x_radian_PID.PID_absolute(Gravity_x_error,0);
		Gyro_y_radian+=Gyro_y_radian_PID.PID_absolute(Gravity_y_error,0);
		Gyro_z_radian+=Gyro_z_radian_PID.PID_absolute(Gravity_z_error,0);
	}



/*一阶龙格库塔法的计算方法*/
#ifndef four_order_Runge_Kutta
	float Q0_temp=Q0;
	float Q1_temp=Q1;
	float Q2_temp=Q2;
	float Q3_temp=Q3;
	Q0+=(-Gyro_x_radian*Q1_temp-Gyro_y_radian*Q2_temp-Gyro_z_radian*Q3_temp)*IMU_cycle/2.0f;
	Q1+=(Gyro_x_radian*Q0_temp+Gyro_z_radian*Q2_temp-Gyro_y_radian*Q3_temp)*IMU_cycle/2.0f;
	Q2+=(Gyro_y_radian*Q0_temp-Gyro_z_radian*Q1_temp+Gyro_x_radian*Q3_temp)*IMU_cycle/2.0f;
	Q3+=(Gyro_z_radian*Q0_temp+Gyro_y_radian*Q1_temp-Gyro_x_radian*Q2_temp)*IMU_cycle/2.0f;
#endif


/*四阶龙格库塔法的计算方法*/
#ifdef four_order_Runge_Kutta
	float Q0_k[4];
	float Q1_k[4];
	float Q2_k[4];
	float Q3_k[4];
	float Q0_temp[4];
	float Q1_temp[4];
	float Q2_temp[4];
	float Q3_temp[4];
	Q0_temp[0]=Q0;
	Q1_temp[0]=Q1;
	Q2_temp[0]=Q2;
	Q3_temp[0]=Q3;
	Q0_k[0]=(-Gyro_x_radian*Q1_temp[0]-Gyro_y_radian*Q2_temp[0]-Gyro_z_radian*Q3_temp[0])/2.0f;
	Q1_k[0]=(Gyro_x_radian*Q0_temp[0]+Gyro_z_radian*Q2_temp[0]-Gyro_y_radian*Q3_temp[0])/2.0f;
	Q2_k[0]=(Gyro_y_radian*Q0_temp[0]-Gyro_z_radian*Q1_temp[0]+Gyro_x_radian*Q3_temp[0])/2.0f;
	Q3_k[0]=(Gyro_z_radian*Q0_temp[0]+Gyro_y_radian*Q1_temp[0]-Gyro_x_radian*Q2_temp[0])/2.0f;
	Q0_temp[1]=Q0_temp[0]+Q0_k[0]*IMU_cycle/2.0f;
	Q1_temp[1]=Q1_temp[0]+Q1_k[0]*IMU_cycle/2.0f;
	Q2_temp[1]=Q2_temp[0]+Q2_k[0]*IMU_cycle/2.0f;
	Q3_temp[1]=Q3_temp[0]+Q3_k[0]*IMU_cycle/2.0f;
	
	Q0_k[1]=(-Gyro_x_radian*Q1_temp[1]-Gyro_y_radian*Q2_temp[1]-Gyro_z_radian*Q3_temp[1])/2.0f;
	Q1_k[1]=(Gyro_x_radian*Q0_temp[1]+Gyro_z_radian*Q2_temp[1]-Gyro_y_radian*Q3_temp[1])/2.0f;
	Q2_k[1]=(Gyro_y_radian*Q0_temp[1]-Gyro_z_radian*Q1_temp[1]+Gyro_x_radian*Q3_temp[1])/2.0f;
	Q3_k[1]=(Gyro_z_radian*Q0_temp[1]+Gyro_y_radian*Q1_temp[1]-Gyro_x_radian*Q2_temp[1])/2.0f;

	Q0_temp[2]=Q0_temp[0]+Q0_k[1]*IMU_cycle/2.0f;
	Q1_temp[2]=Q1_temp[0]+Q1_k[1]*IMU_cycle/2.0f;
	Q2_temp[2]=Q2_temp[0]+Q2_k[1]*IMU_cycle/2.0f;
	Q3_temp[2]=Q3_temp[0]+Q3_k[1]*IMU_cycle/2.0f;

	Q0_k[2]=(-Gyro_x_radian*Q1_temp[2]-Gyro_y_radian*Q2_temp[2]-Gyro_z_radian*Q3_temp[2])/2.0f;
	Q1_k[2]=(Gyro_x_radian*Q0_temp[2]+Gyro_z_radian*Q2_temp[2]-Gyro_y_radian*Q3_temp[2])/2.0f;
	Q2_k[2]=(Gyro_y_radian*Q0_temp[2]-Gyro_z_radian*Q1_temp[2]+Gyro_x_radian*Q3_temp[2])/2.0f;
	Q3_k[2]=(Gyro_z_radian*Q0_temp[2]+Gyro_y_radian*Q1_temp[2]-Gyro_x_radian*Q2_temp[2])/2.0f;	

	Q0_temp[3]=Q0_temp[0]+Q0_k[2]*IMU_cycle;
	Q1_temp[3]=Q1_temp[0]+Q1_k[2]*IMU_cycle;
	Q2_temp[3]=Q2_temp[0]+Q2_k[2]*IMU_cycle;
	Q3_temp[3]=Q3_temp[0]+Q3_k[2]*IMU_cycle;

	Q0_k[3]=(-Gyro_x_radian*Q1_temp[3]-Gyro_y_radian*Q2_temp[3]-Gyro_z_radian*Q3_temp[3])/2.0f;
	Q1_k[3]=(Gyro_x_radian*Q0_temp[3]+Gyro_z_radian*Q2_temp[3]-Gyro_y_radian*Q3_temp[3])/2.0f;
	Q2_k[3]=(Gyro_y_radian*Q0_temp[3]-Gyro_z_radian*Q1_temp[3]+Gyro_x_radian*Q3_temp[3])/2.0f;
	Q3_k[3]=(Gyro_z_radian*Q0_temp[3]+Gyro_y_radian*Q1_temp[3]-Gyro_x_radian*Q2_temp[3])/2.0f;


	Q0=Q0_temp[0]+IMU_cycle*(Q0_k[0]+2*Q0_k[1]+2*Q0_k[2]+Q0_k[3])/6.0f;
	Q1=Q1_temp[0]+IMU_cycle*(Q1_k[0]+2*Q1_k[1]+2*Q1_k[2]+Q1_k[3])/6.0f;
	Q2=Q2_temp[0]+IMU_cycle*(Q2_k[0]+2*Q2_k[1]+2*Q2_k[2]+Q2_k[3])/6.0f;
	Q3=Q3_temp[0]+IMU_cycle*(Q3_k[0]+2*Q3_k[1]+2*Q3_k[2]+Q3_k[3])/6.0f;
	
#endif

//将四元数归一化
	Q_mode=sqrtf(Q0*Q0+Q1*Q1+Q2*Q2+Q3*Q3);
	Q0=Q0/Q_mode;
	Q1=Q1/Q_mode;
	Q2=Q2/Q_mode;
	Q3=Q3/Q_mode;


}

//在更新四元数之后，再根据四元数更新角度
void Class_MPU6500::Angle_update(){



//	Angle_pitch=180.0f*Radian_pitch/PI;
//	Angle_roll=180.0f*Radian_roll/PI;
//	Angle_yaw=180.0f*Radian_yaw/PI;
	Pitch=(180.0f/PI)*asinf(2*(Q1*Q0+Q2*Q3));
	Yaw=(-180.0f/PI)*atan2f(2*(Q1*Q2-Q0*Q3),1-2*Q1*Q1-2*Q3*Q3);
	Roll=(-180.0f/PI)*atan2f(2*(Q1*Q3-Q0*Q2),1-2*Q1*Q1-2*Q2*Q2);

	Radian_pitch=Pitch*PI/180.0f;
	Radian_yaw=Yaw*PI/180.0f;
	Radian_roll=Roll*PI/180.0f;


   if(Yaw-Yaw_last>180.0f){
		Angle_yaw+=Yaw-Yaw_last-360.0f;
   }else if(Yaw-Yaw_last<-180.0f){
		Angle_yaw+=Yaw-Yaw_last+360.0f;
   }else{
		Angle_yaw+=Yaw-Yaw_last;
   }

	if(Roll-Roll_last>180.0f){
		Angle_roll+=Roll-Roll_last-360.0f;
   }else if(Roll-Roll_last<-180.0f){
		Angle_roll+=Roll-Roll_last+360.0f;
   }else{
		Angle_roll+=Roll-Roll_last;
   }

   	if(Pitch-Pitch_last>90.0f){
		Angle_pitch+=Pitch-Pitch_last-180.0f;
   }else if(Pitch-Pitch_last<-90.0f){
		Angle_pitch+=Pitch-Pitch_last+180.0f;
   }else{
		Angle_pitch+=Pitch-Pitch_last;
   }


	Pitch_last=Pitch;
	Roll_last=Roll;
	Yaw_last=Yaw;
}

/*SPI的DMA通讯结束后进行中断数据处理，该函数在SPI中断函数中调用*/
void Class_MPU6500::SPI_Data_process(uint8_t buf[]){




		Acceleration_x_measured=((int16_t)((buf[0]<<8)|(buf[1])))*8.0f*9.8f/32768.0f;
		Gravity_x_measured=Acceleration_x_measured;
		Acceleration_y_measured=((int16_t)((buf[2]<<8)|(buf[3])))*8.0f*9.8f/32768.0f;
		Gravity_y_measured=Acceleration_y_measured;
		Acceleration_z_measured=((int16_t)((buf[4]<<8)|(buf[5])))*8.0f*9.8f/32768.0f;
		Gravity_z_measured=Acceleration_z_measured;
		
		temperature_measured=((int16_t)((buf[6]<<8)|(buf[7])))/333.87f+21.0f;

		Gyro_x_measured=((int16_t)((buf[8]<<8)|(buf[9])))*2000.0f/32768.0f;
		Gyro_y_measured=((int16_t)((buf[10]<<8)|(buf[11])))*2000.0f/32768.0f;
		Gyro_z_measured=((int16_t)((buf[12]<<8)|(buf[13])))*2000.0f/32768.0f;


		Gyro_x_real=gyro_X_Filter.kalman_Filter(Gyro_x_measured)+Gyro_x_error;
		Gyro_y_real=gyro_Y_Filter.kalman_Filter(Gyro_y_measured)+Gyro_y_error;
		Gyro_z_real=gyro_Z_Filter.kalman_Filter(Gyro_z_measured)+Gyro_z_error;

		Gyro_x_radian=PI*Gyro_x_real/180.0f;
		Gyro_y_radian=PI*Gyro_y_real/180.0f;
		Gyro_z_radian=PI*Gyro_z_real/180.0f;



		Gravity_x_guessed=9.8f*2*(Q1*Q3-Q0*Q2);
		Gravity_y_guessed=9.8f*2*(Q3*Q2+Q1*Q0);
		Gravity_z_guessed=9.8f*(1-2*Q1*Q1-2*Q2*Q2);
		temperature_real=Temp_K_Filter.kalman_Filter(temperature_measured);

		//Acceleration_x_real=Mpu_6500.accel_X_Filter.kalman_Filter(Acceleration_x_measured)-Gravity*2*(Q1*Q3-Q0*Q2);
		//Acceleration_y_real=Mpu_6500.accel_Y_Filter.kalman_Filter(Acceleration_y_measured)-Gravity*2*(Q3*Q2+Q1*Q0);
		//Acceleration_z_real=Mpu_6500.accel_Z_Filter.kalman_Filter(Acceleration_z_measured)-Gravity*(1-2*Q1*Q1-2*Q2*Q2);
		Gravity_x_real=accel_X_Filter.kalman_Filter(Acceleration_x_measured,Gravity_x_guessed);
		Gravity_y_real=accel_X_Filter.kalman_Filter(Acceleration_y_measured,Gravity_y_guessed);
		Gravity_z_real=accel_X_Filter.kalman_Filter(Acceleration_z_measured,Gravity_z_guessed);


}







