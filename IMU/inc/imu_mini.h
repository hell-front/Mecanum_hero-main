#ifndef IMU_MINI_H
#define IMU_MINI_H


#include"main.h"
#include"Filter.h"

typedef struct Struct_IMUData_Packet{
	float gyroscope_x;          //unit: rad/s
	float gyroscope_y;          //unit: rad/s
	float gyroscope_z;          //unit: rad/s
	float accelerometer_x;      //m/s^2
	float accelerometer_y;      //m/s^2
	float accelerometer_z;      //m/s^2
	float magnetometer_x;       //mG
	float magnetometer_y;       //mG
	float magnetometer_z;       //mG
	float imu_temperature;      //C
	float Pressure;             //Pa
	float pressure_temperature; //C
	uint32_t Timestamp;          //us
} IMUData_Packet;

typedef struct Struct_AHRSData_Packet
{
	float RollSpeed;   //unit: rad/s
	float PitchSpeed;  //unit: rad/s
	float YawSpeed;//unit: rad/s
	float Roll;        //unit: rad
	float Pitch;       //unit: rad
    float Yaw;     //unit: rad
	float Yaw_degree;     //unit: degree
    float Roll_degree;        //unit: degree
	float Pitch_degree;       //unit: degree

	float Qw;//w          //Quaternion
	float Qx;//x
	float Qy;//y
	float Qz;//z
	uint32_t Timestamp; //unit: us
}AHRSData_Packet;


#ifdef __cplusplus       //-----------��׼д��-----------
extern "C"{                      //-----------��׼д��-----------
#endif






class Class_imu_mini
{
private:
	float Yaw_Degree;//表示的是本次实际输出的陀螺仪的Yaw值，该角度为单圈值
	float Yaw_Degree_last;//表示上一次返回的陀螺仪的Yaw值，大小已经转换为度，该角度为单圈值
	float Yaw_radian_zero;//表示初始上电的时候陀螺仪的Yaw值（取此时车头的指向为Yaw的零点）

public:

	float Angle_Yaw;//，表示实际输出的相对初始时刻的转角，此次时的角度为多圈值，单位为度
	float Angle_Yaw_real;//表示的是Angel——Yaw滤波后的结果，用度来表示，
	float Yaw_speed_real;//表示的是速度低通滤波后的结果，用弧度来表示
	
	uint8_t processed;//表示是否是第一次进入中断函数，初始默认赋值为0，进入一次后改为1

    uint8_t buf_receive[263];
    Struct_IMUData_Packet IMUdata_Packet;
    Struct_AHRSData_Packet AHRSdata_Packet;
    Class_imu_mini();
	void My_Data_processing();//表示将返回的陀螺仪数据进行后续处理，
    uint8_t CRC8_Check(uint8_t* p, uint8_t counter);//负责IMU——MINI的数据的CRC校验，该数据的需要校验的部分只有5个字节
    uint16_t CRC16_Check(uint8_t* p, uint8_t counter);//
    void UART_Data_processing(uint16_t size);//用来标记本次回传数据的长度
    void UART_Data_processing();
    float float32_ARM_TO_x64(float TEMP);
	Class_one_order_lowpass_filter Yaw_Filter;//表示的输出角度的低通滤波，此处会写，但是不一定会用到
	Class_one_order_lowpass_filter Yawspeed_Filter;//表示的输出角速度的低通滤波
};
















#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif






#endif


