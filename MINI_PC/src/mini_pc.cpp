/**
 * @file mini_pc.cpp
 * @author GJC
 * @brief 
 * @version 0.1
 * @date 2023-04-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "mini_pc.h"
#include "referee.h"
#include "gimbal.h"
#include "imu_mini.h"
#include "GM6020.h"
#include "main.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart3;
extern Class_Gimbal Gimbal;
extern Class_imu_mini Imu_mini;
extern GM6020_moter GM6020_pitch;
extern GM6020_moter GM6020_yaw;
extern Referee_System Referee;
/* Private function declarations ---------------------------------------------*/


Class_Mini_pc Data_with_miniPC;
/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 发送minipc需要的信息
 * @note  
 * 
 * @param 
 * @return 
 */
void Class_Mini_pc::Data_processing_to_minipc(void)
{
    
    //uint8_t end_sign = '\n';
    //vx = Gimbal.velocity_yaw - Imu_mini.Yaw_speed_real;
    //memcpy(Mini_pc_tx_buf,&vx,4);
    //vy = Gimbal.velocity_pitch;
    //memcpy(Mini_pc_tx_buf+4,&vy,4);
    //pitch = GM6020_pitch.get_location_real() - GM6020_pitch.location_zero;
    //memcpy(Mini_pc_tx_buf+8,&pitch,4);
    //yaw = GM6020_yaw.get_location_real() - GM6020_yaw.location_zero;
    //memcpy(Mini_pc_tx_buf+12,&yaw,4);
    //id=Referee.Game_robot_status.robot_id;
    //if(Referee.Game_robot_status.robot_id > 50)
    //{
    //    color = 0;
    //}
    //else 
    //{
    //    color = 1;
    //}
    //memcpy(Mini_pc_tx_buf+16,&id,2);
    //if(Gimbal.gimbal_auto == 0)//自瞄状态
    //{
    //    is_auto_aiming_open = 0;
    //}
    //else{
    //    is_auto_aiming_open = 1;
    //}
    //memcpy(Mini_pc_tx_buf+18,&is_auto_aiming_open,1);
    //memcpy(Mini_pc_tx_buf+19,&is_shooting_energy,1);
    //memcpy(Mini_pc_tx_buf+20,&end_sign,1);

    //HAL_UART_Transmit_IT(&huart3,Mini_pc_tx_buf,21);
    uint16_t header = 0xCDAB;
    uint8_t end_sign = '\n';
    memcpy(Mini_pc_tx_buf,&header,2);
    vx = Gimbal.velocity_yaw - Imu_mini.Yaw_speed_real;
    memcpy(Mini_pc_tx_buf+2,&vx,4);
    vy = Gimbal.velocity_pitch;
    memcpy(Mini_pc_tx_buf+6,&vy,4);
    pitch = (GM6020_pitch.get_location_real() - GM6020_pitch.location_zero);
    memcpy(Mini_pc_tx_buf+10,&pitch,4);
    //TODO: Imu_mini.Angle_Yaw_real 调整正负号
    yaw = -(GM6020_yaw.get_location_real() - GM6020_yaw.location_zero + Imu_mini.Angle_Yaw_real); //TODO: 调整正负号
    memcpy(Mini_pc_tx_buf+14,&yaw,4);
    id=Referee.Game_robot_status.robot_id;
    //if(Referee.Game_robot_status.robot_id > 50)
    //{
    //    color = 0;
    //}
    //else 
    //{
    //    color = 1;
    //}
    memcpy(Mini_pc_tx_buf+18,&id,2);
    if(Gimbal.gimbal_auto == 0)//自瞄状态
    {
        is_auto_aiming_open = 0;
    }
    else{
        is_auto_aiming_open = 1;
    }
    memcpy(Mini_pc_tx_buf+20,&is_auto_aiming_open,1);
    memcpy(Mini_pc_tx_buf+21,&is_shooting_energy,1);
    memcpy(Mini_pc_tx_buf+22,&end_sign,1);

    HAL_UART_Transmit_IT(&huart3,Mini_pc_tx_buf,23);
}

/**
 * @brief 接收minipc需要的信息
 * @note  
 * 
 * @param 
 * @return 
 */
void Class_Mini_pc::Data_processing_from_minipc(void)
{
    uint8_t Mini_pc_rx_buf[50];
    HAL_UART_Receive_IT(&huart3,Mini_pc_rx_buf,50);
    memcpy(&delta_yaw,Mini_pc_rx_buf,2);
    memcpy(&delta_pitch,Mini_pc_rx_buf+2,2);
    memcpy(&predicted_delta_yaw,Mini_pc_rx_buf+4,2);
    memcpy(&predicted_delta_pitch,Mini_pc_rx_buf+6,2);
    memcpy(&target_distance,Mini_pc_rx_buf+8,2);
    memcpy(&predicted_vx,Mini_pc_rx_buf+10,2);
    memcpy(&predicted_vy,Mini_pc_rx_buf+12,2);
    memcpy(&msgflag,Mini_pc_rx_buf+14,2);
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/








