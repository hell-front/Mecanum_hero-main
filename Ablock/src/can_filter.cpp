#include"can_filter.h"
#include"main.h"
#include"stdio.h"





/*16位列表模式，此模式下最后两个ID都是可以接受的帧头*/
void can_filter_list16_init(CAN_HandleTypeDef *hcan,uint32_t Bank,uint32_t canID_mode,uint32_t FIFO_number,uint32_t FilterID,uint32_t FilterMaskID){
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank=Bank;
    sFilterConfig.FilterMode=CAN_FILTERMODE_IDLIST;
    sFilterConfig.FilterScale=CAN_FILTERSCALE_16BIT;

    sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;
    sFilterConfig.FilterFIFOAssignment=FIFO_number;

    sFilterConfig.FilterIdHigh=(((FilterID>>16)<<5)&0xffff);
    sFilterConfig.FilterIdLow=(FilterID<<5)&0xffff;
    sFilterConfig.FilterMaskIdHigh=(((FilterMaskID>>16)<<5)&0xffff);
    sFilterConfig.FilterMaskIdLow=(FilterMaskID<<5)&0xffff;

    sFilterConfig.SlaveStartFilterBank=14;

    if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
        printf("Filter_list16_Error!");    
    }


}

/*32位列表模式*/
void can_filter_list32_init(CAN_HandleTypeDef *hcan,uint32_t Bank,uint32_t canID_mode,uint32_t FIFO_number,uint32_t FilterID,uint32_t FilterMaskID){

        CAN_FilterTypeDef sFilterConfig;

        sFilterConfig.FilterBank=Bank;
        sFilterConfig.FilterMode=CAN_FILTERMODE_IDLIST;
        sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;

        sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;
        sFilterConfig.FilterFIFOAssignment=FIFO_number;
        if(canID_mode==CAN_ID_STD){

            sFilterConfig.FilterIdHigh=(FilterID<<5)&0xffff;
            sFilterConfig.FilterIdLow=0|(CAN_ID_STD|CAN_RTR_DATA);
            sFilterConfig.FilterMaskIdHigh=(FilterMaskID<<5)&0xffff;
            sFilterConfig.FilterMaskIdLow=0|(CAN_ID_STD|CAN_RTR_DATA);

        }else{
            sFilterConfig.FilterIdHigh=((FilterID<<3)>>16)&0xffff;
            sFilterConfig.FilterIdLow=((FilterID<<3)&0xffff)|(CAN_ID_EXT|CAN_RTR_DATA);
            sFilterConfig.FilterIdHigh=((FilterMaskID<<3)>>16)&0xffff;
            sFilterConfig.FilterIdLow=((FilterMaskID<<3)&0xffff)|(CAN_ID_EXT|CAN_RTR_DATA);//确定可接受的是拓展帧，数据帧
        }

        sFilterConfig.SlaveStartFilterBank=14;

        if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
            printf("Filter_list32_Error!");    
        }
		
		

}




/*16位掩码模式*/
void can_filter_mask16_init(CAN_HandleTypeDef *hcan,uint32_t Bank,uint32_t canID_mode,uint32_t FIFO_number,uint32_t FilterID,uint32_t FilterMaskID){

        CAN_FilterTypeDef sFilterConfig;

        sFilterConfig.FilterBank=Bank;
        sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
        sFilterConfig.FilterScale=CAN_FILTERSCALE_16BIT;

        sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;
        sFilterConfig.FilterFIFOAssignment=FIFO_number;

        sFilterConfig.FilterIdHigh=(((FilterID>>16)<<5)&0xffff)|CAN_RTR_DATA;//确定可以接受的是数据帧
        sFilterConfig.FilterIdLow=((FilterID<<5)&0xffff)|CAN_RTR_DATA;
        sFilterConfig.FilterMaskIdHigh=(((FilterMaskID>>16)<<5)&0xffff)|0x10;//确定可以接受的是数据帧（0表示的是数据帧）
        sFilterConfig.FilterMaskIdLow=((FilterMaskID<<5)&0xffff)|0x10;

        sFilterConfig.SlaveStartFilterBank=14;

        if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
            printf("Filter_mask16_Error!\n");    
        }
}


/*32位掩码模式*/
void can_filter_mask32_init(CAN_HandleTypeDef *hcan,uint32_t Bank,uint32_t canID_mode,uint32_t FIFO_number,uint32_t FilterID,uint32_t FilterMaskID){

        CAN_FilterTypeDef sFilterConfig;

         sFilterConfig.FilterBank=Bank;
        sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
        sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;

        sFilterConfig.FilterActivation=CAN_FILTER_ENABLE;
        sFilterConfig.FilterFIFOAssignment=FIFO_number;

        if(canID_mode==CAN_ID_STD){

            sFilterConfig.FilterIdHigh=(FilterID<<5)&0xffff;
            sFilterConfig.FilterIdLow=0|(CAN_ID_STD|CAN_RTR_DATA);//确保接受的是数据帧
            sFilterConfig.FilterMaskIdHigh=(FilterMaskID<<5)&0xffff;
            sFilterConfig.FilterMaskIdLow=0|0x02;//确保接受的是数据帧

					

        }else{
            sFilterConfig.FilterIdHigh=((FilterID<<3)>>16)&0xffff;
            sFilterConfig.FilterIdLow=((FilterID<<3)&0xffff)|(CAN_ID_EXT|CAN_RTR_DATA);//确保接受的是数据帧
            sFilterConfig.FilterMaskIdHigh=((FilterMaskID<<3)>>16)&0xffff;
            sFilterConfig.FilterMaskIdLow=((FilterMaskID<<3)&0xffff)|0x02;//确保接受的是数据帧
						
        }

        sFilterConfig.SlaveStartFilterBank=14;

        if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK){
            printf("Filter_mask32_Error!\n");    
        }
		
}



