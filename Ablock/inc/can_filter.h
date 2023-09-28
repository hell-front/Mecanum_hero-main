#ifndef _CAN_FILTER_H
#define _CAN_FILTER_H



#include"main.h"
#include"stdio.h"

#ifdef __cplusplus       //-----------标准写法-----------
extern "C"{                      //-----------标准写法-----------
#endif

void can_filter_list16_init(CAN_HandleTypeDef *hcan,uint32_t Bank,uint32_t canID_mode,uint32_t FIFO_number,uint32_t FilterID,uint32_t FilterMaskID);



void can_filter_list32_init(CAN_HandleTypeDef *hcan,uint32_t Bank,uint32_t canID_mode,uint32_t FIFO_number,uint32_t FilterID,uint32_t FilterMaskID);





void can_filter_mask16_init(CAN_HandleTypeDef *hcan,uint32_t Bank,uint32_t canID_mode,uint32_t FIFO_number,uint32_t FilterID,uint32_t FilterMaskID);

void can_filter_mask32_init(CAN_HandleTypeDef *hcan,uint32_t Bank,uint32_t canID_mode,uint32_t FIFO_number,uint32_t FilterID,uint32_t FilterMaskID);

	
#ifdef __cplusplus      //-----------标准写法-----------
}                                          //-----------标准写法-----------
#endif 	
	
	
	
#endif

