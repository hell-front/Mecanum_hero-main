#ifndef MPU6500_H
#define MPU6500_H

#ifdef __cplusplus       //-----------��׼д��-----------
extern "C"{                      //-----------��׼д��-----------
#endif


#include "main.h"
#include "PID.h"
#include "Filter.h"




#define MPU6500_SELF_TEST_XG        (0x00)
#define MPU6500_SELF_TEST_YG        (0x01)
#define MPU6500_SELF_TEST_ZG        (0x02)
#define MPU6500_SELF_TEST_XA        (0x0D)
#define MPU6500_SELF_TEST_YA        (0x0E)
#define MPU6500_SELF_TEST_ZA        (0x0F)

/*此六�??寄存器用于消除陀螺仪输出�??的直流偏�??。在进入传感器寄存器之前，将此寄存器�??的值添加到陀螺仪传感器值中�??*/
#define MPU6500_XG_OFFSET_H         (0x13)
#define MPU6500_XG_OFFSET_L         (0x14)
#define MPU6500_YG_OFFSET_H         (0x15)
#define MPU6500_YG_OFFSET_L         (0x16)
#define MPU6500_ZG_OFFSET_H         (0x17)
#define MPU6500_ZG_OFFSET_L         (0x18)

/*除以内部采样�??(see register CONFIG)生成控制传感器数�??输出速率的采样率，FIFO采样�??.此寄存器�??有在FCHOICE=2‘b11(FCHOICE_B寄存器位�??2’b00)�??(0<DLPF_CFG<7)时才有效
采样�??=内部采样�??/(1+SMPLRT_DIV),内部采样�??=1 kHz*/
#define MPU6500_SMPLRT_DIV          (0x19)

/*四个配置寄存�?�??�明如下文所�??*/
#define MPU6500_CONFIG              (0x1A)
#define MPU6500_GYRO_CONFIG         (0x1B)
#define MPU6500_ACCEL_CONFIG        (0x1C)
#define MPU6500_ACCEL_CONFIG_2      (0x1D)

/*低功率加速度�??ODR控制寄存�??*/
#define MPU6500_LP_ACCEL_ODR        (0x1E)

/*此寄存器保存x/y/z�??�??返回�??*/
#define MPU6500_MOT_THR             (0x1F)

/*FIFO使能寄存器。若�??1，则将�?�应数据以采样�?�率写入FIFO*/
#define MPU6500_FIFO_EN             (0x23)

/*IIC主�?��?�控制器，�?�下�??*/
#define MPU6500_I2C_MST_CTRL        (0x24)

/*IIC从�?��?�相关寄存器*/
#define MPU6500_I2C_SLV0_ADDR       (0x25)
#define MPU6500_I2C_SLV0_REG        (0x26)
#define MPU6500_I2C_SLV0_CTRL       (0x27)
#define MPU6500_I2C_SLV1_ADDR       (0x28)
#define MPU6500_I2C_SLV1_REG        (0x29)
#define MPU6500_I2C_SLV1_CTRL       (0x2A)
#define MPU6500_I2C_SLV2_ADDR       (0x2B)
#define MPU6500_I2C_SLV2_REG        (0x2C)
#define MPU6500_I2C_SLV2_CTRL       (0x2D)
#define MPU6500_I2C_SLV3_ADDR       (0x2E)
#define MPU6500_I2C_SLV3_REG        (0x2F)
#define MPU6500_I2C_SLV3_CTRL       (0x30)
#define MPU6500_I2C_SLV4_ADDR       (0x31)
#define MPU6500_I2C_SLV4_REG        (0x32)
#define MPU6500_I2C_SLV4_DO         (0x33)
#define MPU6500_I2C_SLV4_CTRL       (0x34)
#define MPU6500_I2C_SLV4_DI         (0x35)

/*IIC  主�?��?�状态寄存器*/
#define MPU6500_I2C_MST_STATUS      (0x36)
/*三个�??�??相关寄存�??*/
#define MPU6500_INT_PIN_CFG         (0x37)
#define MPU6500_INT_ENABLE          (0x38)
#define MPU6500_INT_STATUS          (0x3A)

/*�??14�??寄存器存储加速度、陀螺仪、温度的原�?�数�??*/
#define MPU6500_ACCEL_XOUT_H        (0x3B)
#define MPU6500_ACCEL_XOUT_L        (0x3C)
#define MPU6500_ACCEL_YOUT_H        (0x3D)
#define MPU6500_ACCEL_YOUT_L        (0x3E)
#define MPU6500_ACCEL_ZOUT_H        (0x3F)
#define MPU6500_ACCEL_ZOUT_L        (0x40)
#define MPU6500_TEMP_OUT_H          (0x41)
#define MPU6500_TEMP_OUT_L          (0x42)
#define MPU6500_GYRO_XOUT_H         (0x43)
#define MPU6500_GYRO_XOUT_L         (0x44)
#define MPU6500_GYRO_YOUT_H         (0x45)
#define MPU6500_GYRO_YOUT_L         (0x46)
#define MPU6500_GYRO_ZOUT_H         (0x47)
#define MPU6500_GYRO_ZOUT_L         (0x48)

/*�??24�??寄存器存储IIC从�?��?�（0�??1�??2�??3)通过辅助IIC接口，从外部传感�?�??�取的数�??
从机设�??4读取的数�??存放在I2C_SLV4_DI�??（寄存器53�??*/
#define MPU6500_EXT_SENS_DATA_00    (0x49)
#define MPU6500_EXT_SENS_DATA_01    (0x4A)
#define MPU6500_EXT_SENS_DATA_02    (0x4B)
#define MPU6500_EXT_SENS_DATA_03    (0x4C)
#define MPU6500_EXT_SENS_DATA_04    (0x4D)
#define MPU6500_EXT_SENS_DATA_05    (0x4E)
#define MPU6500_EXT_SENS_DATA_06    (0x4F)
#define MPU6500_EXT_SENS_DATA_07    (0x50)
#define MPU6500_EXT_SENS_DATA_08    (0x51)
#define MPU6500_EXT_SENS_DATA_09    (0x52)
#define MPU6500_EXT_SENS_DATA_10    (0x53)
#define MPU6500_EXT_SENS_DATA_11    (0x54)
#define MPU6500_EXT_SENS_DATA_12    (0x55)
#define MPU6500_EXT_SENS_DATA_13    (0x56)
#define MPU6500_EXT_SENS_DATA_14    (0x57)
#define MPU6500_EXT_SENS_DATA_15    (0x58)
#define MPU6500_EXT_SENS_DATA_16    (0x59)
#define MPU6500_EXT_SENS_DATA_17    (0x5A)
#define MPU6500_EXT_SENS_DATA_18    (0x5B)
#define MPU6500_EXT_SENS_DATA_19    (0x5C)
#define MPU6500_EXT_SENS_DATA_20    (0x5D)
#define MPU6500_EXT_SENS_DATA_21    (0x5E)
#define MPU6500_EXT_SENS_DATA_22    (0x5F)
#define MPU6500_EXT_SENS_DATA_23    (0x60)

/*IIC从�?��?�数�??输出寄存�??*/
#define MPU6500_I2C_SLV0_DO         (0x63)
#define MPU6500_I2C_SLV1_DO         (0x64)
#define MPU6500_I2C_SLV2_DO         (0x65)
#define MPU6500_I2C_SLV3_DO         (0x66)

#define MPU6500_I2C_MST_DELAY_CTRL  (0x67)
#define MPU6500_SIGNAL_PATH_RESET   (0x68)
#define MPU6500_MOT_DETECT_CTRL     (0x69)
#define MPU6500_USER_CTRL           (0x6A)

/*电源管理寄存�??，用于配置MPU6500时钟源，控制传感器失能等*/
#define MPU6500_PWR_MGMT_1          (0x6B)
#define MPU6500_PWR_MGMT_2          (0x6C)

/*记录写入到FIFO的字节数*/
#define MPU6500_FIFO_COUNTH         (0x72)
#define MPU6500_FIFO_COUNTL         (0x73)

/*用于从FIFO缓冲区�?�写数据*/
#define MPU6500_FIFO_R_W            (0x74)

/*存储一�??8位数�??，用于验证�?��?�的标示*/
#define MPU6500_WHO_AM_I            (0x75)	// mpu6500 id = 0x70

/*此六�??寄存器用于消除加速度计输出中的直流偏�??。在进入传感器寄存器之前，将此寄存器�??的值添加到加速度计传感器值中�??*/
#define MPU6500_XA_OFFSET_H         (0x77)
#define MPU6500_XA_OFFSET_L         (0x78)
#define MPU6500_YA_OFFSET_H         (0x7A)
#define MPU6500_YA_OFFSET_L         (0x7B)
#define MPU6500_ZA_OFFSET_H         (0x7D)
#define MPU6500_ZA_OFFSET_L         (0x7E)
	
#define MPU6500_ID					(0x70)	
#define MPU_IIC_ADDR				(0x68)



//———————————————————————————————————————————�?

#define  Error_number                (50)
#define  IMU_Time                    (5)
#define  IMU_cycle                   (0.005f)//���?0.005s����һ�Σ�ΪIMU_Time��ǧ��֮һ
#define  four_order_Runge_Kutta


#define PI 3.1415926f





void MPU_6500_init();

void MPU_6500_Read_DMA();
void MPU_6500_Read_Temperature();
void MPU_6500_Read_gyro();
void MPU_6500_Read_accel();
void MPU_6500_Data_process(uint8_t buf[]);

void MPU6500_get_errorG();
void MPU_6500_static_errorG_Eliminate();




class Class_MPU6500{
    
    private:
        float Acceleration_x_measured;
        float Acceleration_y_measured;
        float Acceleration_z_measured;

        float Gyro_x_measured;
        float Gyro_y_measured;
        float Gyro_z_measured;

 



        float temperature_measured;



    public:
 


        float Gravity_x_measured;
        float Gravity_y_measured;
        float Gravity_z_measured;
        float Gravity_measured;//重力加速度测量值

        float Gravity_x_guessed;
        float Gravity_y_guessed;
        float Gravity_z_guessed;
        float Gravity_guessed;//重力加速度测量值

        float Gravity_x_real;
        float Gravity_y_real;
        float Gravity_z_real;
        float Gravity_real;//卡尔曼融合算法后的最置信结果当作真实值

        float Gravity_x_error;
        float Gravity_y_error;
        float Gravity_z_error;//重力加速度的误差，此处实际中并未用到


        float Acceleration_x_real;
        float Acceleration_y_real;
        float Acceleration_z_real;



        float Gyro_x_error;//角速度的零漂,单位为°/s
        float Gyro_y_error;
        float Gyro_z_error;


        float Gyro_x_real;//角速度去零漂后的并滤波的真实值
        float Gyro_y_real;
        float Gyro_z_real;

        float Gyro_x_radian;//角速度以弧度/s表示的结果
        float Gyro_y_radian;
        float Gyro_z_radian;

        float Angle_roll;//以角度表示的姿态角
        float Angle_pitch;
        float Angle_yaw;

        float Roll;
        float Yaw;
        float Pitch;

        float Roll_last;
        float Yaw_last;
        float Pitch_last;



        float Radian_roll;//以弧度表示的姿态角
        float Radian_pitch;
        float Radian_yaw;

        void Angle_update();//角度更新函数


        

        float Q0;//表示姿态的四元数
        float Q1;
        float Q2;
        float Q3;

        void Q_init();//��̬����ϵ��ʼ��������һ���Գ�ǰ��Ϊx�ᣬxyƽ����ˮƽ��������?
        void Q_update();//���ݽ��ٶȵ���ֵ��������Ԫ��
        

        float temperature_real;//�¶ȼƿ������˲�����ʵֵ
        
        float temperature_target;//�¶ȼƵ��趨ֵ��ͨ��Ϊ45�浽50��֮��


        Class_PID Gyro_x_radian_PID;
        Class_PID Gyro_y_radian_PID;
        Class_PID Gyro_z_radian_PID;
        Class_PID temperature_PID;//�¶ȼƵ�PID���㣬������ΪPWM



        Class_Kalman_filter gyro_X_Filter;
        Class_Kalman_filter gyro_Y_Filter;
        Class_Kalman_filter gyro_Z_Filter;

        Class_Kalman_filter accel_X_Filter; 
        Class_Kalman_filter accel_Y_Filter; 
        Class_Kalman_filter accel_Z_Filter; 

        Class_Kalman_filter Temp_K_Filter;








        void MPU6500_config();

        void set_IMU_temp_to_target();



        void set_acceleration_X_measured(float ACCELERATION){Acceleration_x_measured=ACCELERATION;};
        float get_acceleration_X_measured(){return Acceleration_x_measured;}
        void set_acceleration_Y_measured(float ACCELERATION){Acceleration_y_measured=ACCELERATION;};
        float get_acceleration_Y_measured(){return Acceleration_y_measured;}
        void set_acceleration_Z_measured(float ACCELERATION){Acceleration_z_measured=ACCELERATION;};
        float get_acceleration_Z_measured(){return Acceleration_z_measured;}

        void set_gyro_X_measured(float GYRO){Gyro_x_measured=GYRO;}
        float get_gyro_X_measured(){return Gyro_x_measured;}
        void set_gyro_Y_measured(float GYRO){Gyro_y_measured=GYRO;}
        float get_gyro_Y_measured(){return Gyro_y_measured;}
        void set_gyro_Z_measured(float GYRO){Gyro_z_measured=GYRO;}
        float get_gyro_Z_measured(){return Gyro_z_measured;}




        void set_temperature_measured(float TEMPERATURE){temperature_measured=TEMPERATURE;}
        float get_temperature_measured(){return temperature_measured;}

        void SPI_Data_process(uint8_t buf[]);

 

            




};








#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif 



#endif
