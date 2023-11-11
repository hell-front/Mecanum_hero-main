#include "main.h"
#include "stm32f4xx_it.h"
#include "callback.h"
#include "referee.h"
#include "stdio.h"
#include "remote.h"
#include "moter.h"
#include "Ablock.h"
#include "task.h"
#include "Chassis.h"
#include "gimbal.h"
#include"imu_mini.h"
#include "mini_pc.h"
#include "Pan_Tilt_AHRS.h"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DMA_HandleTypeDef hdma_spi5_rx;
extern DMA_HandleTypeDef hdma_spi5_tx;
extern SPI_HandleTypeDef hspi5;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart8;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim13;

/*----------在Chassis.cpp文件中定义的变量----------*/
extern C620_driver C620_chassis_1;
extern C620_driver C620_chassis_2;
extern C620_driver C620_chassis_3;
extern C620_driver C620_chassis_4;

extern Class_Super_Cup Super_Cup;
/*----------在gimbal.cpp文件中定义的变量----------*/
extern GM6020_moter GM6020_pitch;
extern GM6020_moter GM6020_yaw;
extern uint8_t auto_aim_buf[];
extern Class_Gimbal Gimbal;

/*----------在shoot.cpp文件中定义的变量----------*/
extern C620_driver C620_plate;
extern C620_driver friction_left_front;
extern C620_driver friction_right_front;
extern C620_driver friction_left_back;
extern C620_driver friction_right_back;


/*----------在referee.cpp文件中定义的变量----------*/
extern Referee_System Referee;
extern uint8_t referee_buf[];



extern Class_Remote_data Remote;

/*----------在mini_pc.cpp文件中定义的变量----------*/
extern Class_Mini_pc Data_with_miniPC;



extern Class_MPU6500 Mpu_6500;

extern Class_imu_mini Imu_mini;


extern rt_sem_t sem_mini_pc;
extern uint8_t remote_buf[18];


extern rt_sem_t sem_can_Tx_full;





void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){
      if(hcan->Instance==CAN1){
        rt_sem_release(sem_can_Tx_full);
      }
      if(hcan->Instance==CAN2){
        rt_sem_release(sem_can_Tx_full);
      }

}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){
      if(hcan->Instance==CAN1){
        rt_sem_release(sem_can_Tx_full);
      }
      if(hcan->Instance==CAN2){
        rt_sem_release(sem_can_Tx_full);
      }


}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){
      if(hcan->Instance==CAN1){
        rt_sem_release(sem_can_Tx_full);
      }
      if(hcan->Instance==CAN2){
        rt_sem_release(sem_can_Tx_full);
      }

}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
      
      CAN_RxHeaderTypeDef can_Rx_header;
      uint8_t can_Rx_buf[8];
      
      HAL_CAN_GetRxMessage(hcan,CAN_FilterFIFO0,&can_Rx_header,can_Rx_buf);
      if(hcan->Instance==CAN1){
        switch (can_Rx_header.StdId)
        {
        case 0x201:{
            C620_chassis_1.Can_Data_processing(can_Rx_buf);
            break;
        }
        case 0x202:{
            C620_chassis_2.Can_Data_processing(can_Rx_buf);
            break;
        }
        case 0x203:{
            C620_chassis_3.Can_Data_processing(can_Rx_buf);
            break;
        }
        case 0x204:{
            C620_chassis_4.Can_Data_processing(can_Rx_buf);
            break;
        }
        case 0x206:{
            
            break;  
        }
        case 0x207:{
            
            break;
        }
        case 0x210:{
          Super_Cup.Can_Data_processing(can_Rx_buf);
          break;
        }
        default:
          break;
        }

      }
      if (hcan->Instance==CAN2)
      {
				switch (can_Rx_header.StdId)
					{
					case 0x202:{
						C620_plate.Can_Data_processing(can_Rx_buf);
						break;
					}
					case 0x204:{
						friction_left_front.Can_Data_processing(can_Rx_buf);
						break;

					}
					case 0x203:{
						friction_right_front.Can_Data_processing(can_Rx_buf);
						break;

					}
					case 0x205:{
						GM6020_yaw.Can_Data_processing(can_Rx_buf);
						break;
					}
					case 0x206:{
						GM6020_pitch.Can_Data_processing(can_Rx_buf);
						break;
					}
          case 0x208:{
						friction_left_back.Can_Data_processing(can_Rx_buf);
						break;

					}
					case 0x207:{
						friction_right_back.Can_Data_processing(can_Rx_buf);
						break;

					}
          case 0x210:{
						friction_right_back.Can_Data_processing(can_Rx_buf);
						break;

					}
					default:
						break;
					}
        
      }
      


      
}
/*因不明原因使用FIFO1时程序会发生错误，故暂时不用FIFO1*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
      CAN_RxHeaderTypeDef can_Rx_header;
      uint8_t can_Rx_buf[8];
      
      HAL_CAN_GetRxMessage(hcan,CAN_FilterFIFO0,&can_Rx_header,can_Rx_buf);
      if(hcan->Instance==CAN1){

      }
      if (hcan->Instance==CAN2)
      {

        
      }


}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){


  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  if(htim->Instance==TIM13){
    //Mpu_6500.Q_update();
    //Mpu_6500.Angle_update();
  }

}


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//  if(huart->Instance==huart2.Instance){
//    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
//    printf("a");
//  }
//}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
        static uint16_t test_i = 0;
        if(huart->Instance==USART2){
            HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,GPIO_PIN_RESET);
            
        }
        if(huart->Instance==USART1){
            Remote.remote_data_processing();
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1,remote_buf,18);
        }
        if(huart->Instance==USART6){
          Referee.UART_Data_processing(referee_buf,Size);
          memset(referee_buf, 0, REFER_NUM_MAX);
          HAL_UARTEx_ReceiveToIdle_DMA(&huart6,referee_buf,REFER_NUM_MAX);
        }
        if(huart->Instance==USART3){
          Gimbal.UART_Data_processing(auto_aim_buf);
          //rt_sem_release(sem_mini_pc);
          Data_with_miniPC.process=1;
          Data_with_miniPC.Data_processing_to_minipc();

          HAL_UARTEx_ReceiveToIdle_IT(&huart3,auto_aim_buf,18);
        }
        if(huart->Instance==UART7){
          Imu_mini.UART_Data_processing(Size);
          if(test_i<50)
          {
            test_i++;
          }
          else
          {
            test_i = 0;
            HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_14);
          }
          
          HAL_UARTEx_ReceiveToIdle_DMA(&huart7,Imu_mini.buf_receive,263);
        }
        if(huart->Instance==UART8){
          Pan_Tilt_AHRS.DataCapture(Pan_Tilt_AHRS_Rx_Buff, 11);
          HAL_UARTEx_ReceiveToIdle_IT(&huart8,Pan_Tilt_AHRS_Rx_Buff,Pan_Tilt_AHRS_RX_BUFFER_SIZE);
        }
}


extern uint8_t MPU_6500_rx_buf[14];

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
      if(hspi->Instance==hspi5.Instance){
        //HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_14);
        Mpu_6500.SPI_Data_process(MPU_6500_rx_buf);
        HAL_GPIO_WritePin(GPIOF,GPIO_PIN_6,GPIO_PIN_SET);
      
      }
}


