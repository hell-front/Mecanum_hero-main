#include "LED.h"
#include "main.h"


/*跑马灯函数1，效果为8个灯依次亮灭*/
void Galloping(){
        static uint8_t led_label=0;
        switch (led_label)
        {
        case 1:{
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_8,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);
            led_label=2;
            break;
        }
        case 2:{
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
            led_label=3;
            break;
        }
        case 3:{
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_RESET);
            led_label=4;
            break;
        }
        case 4:{
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_RESET);
            led_label=5;
            break;
        }
        case 5:{
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_4,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,GPIO_PIN_RESET);
            led_label=6;
            break;
        }                  
        case 6:{
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_5,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
            led_label=7;
            break;
        }
        case 7:{
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_RESET);
            led_label=8;
            break;
        }                    
        case 8:{
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_8,GPIO_PIN_RESET);
            led_label=1;
            break;
        }        
        default:{
            led_label=1;
        }
            break;
        }
}

