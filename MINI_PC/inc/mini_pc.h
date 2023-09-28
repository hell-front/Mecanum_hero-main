/**
 * @file mini_pc.h
 * @author GJC
 * @brief 
 * @version 0.1
 * @date 2023-04-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef  _MINI_PC_H_
#define  _MINI_PC_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Exported macros -----------------------------------------------------------*/
#ifdef __cplusplus       //-----------��׼д��-----------
extern "C"{                      //-----------��׼д��-----------
#endif
/* Exported types ------------------------------------------------------------*/
class Class_Mini_pc{

    public:
        uint8_t process;//表示是否第一次接收到数据
        //发送的数据
        uint8_t Mini_pc_tx_buf[50];//发送数据的缓存
        float vy;       // 当前pitch velocity 单位-18000~18000对应-180°~180°，用云台电机反馈的角度-零点（待测）
        float vx;       // 当前yaw velocity，单位-18000~18000对应-180°~180°
        float pitch;    // pitch angle
        float yaw;      // yaw angle 
        // int16_t v1;  // 相对视角的左右方向速度，1m/s对应1000
        // int16_t v2;  // 相对视角的前进方向速度，1m/s对应1000
        uint16_t id;                 //自己的ID
        bool is_auto_aiming_open;   // 是否开启自瞄
        bool is_shooting_energy;    // 是否击打能量机关
        
        /*可能需要的标志：自身是否处于小陀螺状态*/
        // uint16_t msgflag;	    // 从低位开始
        // 下位机发送给妙算的数据必须以\n结尾
        //接收的数据，目前版本接受的数据定义在gimbal.cpp里面
        /*目标偏差的角度*/
        int16_t delta_yaw;      // 单位-18000~18000对应-180°~180°
        int16_t delta_pitch;    // 单位-18000~18000对应-180°~180°
        int16_t predicted_delta_yaw;
        int16_t predicted_delta_pitch;
        /*目标的距离*/
        int16_t target_distance;// 单位1000对应1米
        int16_t predicted_vx;
        int16_t predicted_vy; 
        /*标志位
            msgflag[15] 有无识别到目标
        */
        uint16_t msgflag;
        
        /*可能需要的变量*/
        // int16_t fire_delay;  // 下次发射的延迟 单位毫秒
        void Data_processing_to_minipc(void);
        void Data_processing_from_minipc(void);


};
/* Exported variables --------------------------------------------------------*/
#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif
/* Exported function declarations --------------------------------------------*/

#endif  
