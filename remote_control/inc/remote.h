#ifndef REMOTE_H
#define REMOTE_H

#ifdef __cplusplus       //-----------��׼д��-----------
extern "C"{                      //-----------��׼д��-----------
#endif


#include "main.h"

#define Gain_x  (5.0f)
#define Gain_y  (5.0f)
#define Gain_omega (-20.0f)
#define Gain_pitch (-0.0003f)
#define Gain_yaw   (0.001f)
#define Gain_theta (0.05f)
#define Gain_friction  (0.1f)
	
#define MOUSE_GAIN_YAW  (0.03f)
#define MOUSE_GAIN_PITCH (0.01f)

#define Remote_Switch_Left1   (1u)//OFF档位 从上往下数挡位依次为123
#define Remote_Switch_Left2   (3u)//CL档位 从上往下数挡位依次为123
#define Remote_Switch_Left3   (2u)//HL档位 从上往下数挡位依次为123

#define Remote_Switch_Right1  (1u)//GPS档位 从上往下数挡位依次为123
#define Remote_Switch_Right2  (3u)//ATTI档位 从上往下数挡位依次为123
#define Remote_Switch_Right3  (2u)//ATTI档位 从上往下数挡位依次为123





#define Chassis_v_x_max   (3300)//x方向速度最大值
#define Chassis_v_y_max   (3300)//y方向速度最大值
#define Chassis_v_omega_max   (10000) //小陀螺模式的自转速度

class Class_Remote_data
{

private:

    uint32_t State_right1_num;//表示右拨码连续进入的时间，连续进入2s（达到142）则进入连射状态，如果摩擦轮不在启动状态，则无法进入该状态
	uint32_t Mouse_left_num;//表示左键按下的时间，连续进入1s（达到71）则进入连射状态
	uint32_t friction_num;//表示摩擦轮转动时间，发射停止3s后关闭摩擦轮；
public:
    Class_Remote_data();

    int16_t Channel_0;
    int16_t Channel_1;
    int16_t Channel_2;
    int16_t Channel_3;
    uint8_t State_left;
    uint8_t State_right;

    uint8_t State_left_last;
    uint8_t State_right_last;


    int16_t Mouse_x;
    int16_t Mouse_y;
    int16_t Mouse_z;
    uint8_t Mouse_left;
    uint8_t Mouse_right;
	uint8_t Mouse_right_last;//鼠标右键前一状态
	uint8_t Mouse_left_last;//鼠标左键的前一状态
	uint8_t Key_shift_last;
	uint8_t Key_ctrl_last;
	uint8_t Key_Z_last;
	uint8_t Key_X_last;
	uint8_t Key_C_last;
	uint8_t Key_R_last;
	uint8_t Key_Q_last;
    uint8_t Key_E_last;
	uint8_t Key_F_last;
	uint8_t Gyro_state;//小陀螺模式0关闭1开启


    uint16_t keyboard;

    uint8_t Key_W;//用于检测键盘各个按键是否被按下
    uint8_t Key_A;
    uint8_t Key_S;
    uint8_t Key_D;
    uint8_t Key_Q;
    uint8_t Key_E;
    uint8_t Key_Shift;
    uint8_t Key_Ctrl;
	uint8_t Key_R;
	uint8_t Key_F;
	uint8_t Key_G;
	uint8_t Key_Z;
	uint8_t Key_X;
	uint8_t Key_C;
	uint8_t Key_V;
	uint8_t Key_B;
    int16_t Mouse_x_last;
    int16_t Mouse_y_last;

    int16_t Channel_user;

    int16_t connected;//表示遥控器是否连接 

    int16_t update;//表示遥控器数据是否更新，如果更新，则表示已连接上，如果没有，则表示未连接，connexted置0

    void remote_data_processing();
    void remote_keyboard_control();
    void remote_DT7_control();

};
















#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif 



#endif
