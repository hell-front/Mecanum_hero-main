/**
 * @file Servo.cpp
 * @author gjc
 * @brief 
 * @version 0.1
 * @date 2023-04-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */

/* Includes ------------------------------------------------------------------*/
#include "Servo.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 设置舵机旋转角度
 * @note  angle 旋转角度 选用TIM2 CH1 装载数2000
 * 			0.5ms 90 1ms 45 1.5ms 0 2ms 45 2.5ms 90 T = 20ms 
 * @param 
 * @return 
 */
void set_servo_revolve_angle(uint16_t pwmval)
{
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//打开PWM
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,pwmval);
	
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

