#include "moter.h"
#include "main.h"







void Send_Data_Dj(CAN_HandleTypeDef *hcan,uint16_t ID,int16_t Control_value1,int16_t Control_value2,int16_t Control_value3,int16_t Control_value4){
            CAN_TxHeaderTypeDef TX_header;
            uint8_t Txbuf[8];
            uint32_t TxMAailBox;
          
            TX_header.DLC=8;

            TX_header.StdId=ID;
            TX_header.IDE=CAN_ID_STD;
            TX_header.RTR=CAN_RTR_DATA;


            Txbuf[1]=Control_value1&0xff;
            Txbuf[0]=(Control_value1>>8)&0xff;
            Txbuf[3]=Control_value2&0xff;
            Txbuf[2]=(Control_value2>>8)&0xff;
            Txbuf[5]=Control_value3&0xff;
            Txbuf[4]=(Control_value3>>8)&0xff;
            Txbuf[7]=Control_value4&0xff;
            Txbuf[6]=(Control_value4>>8)&0xff;

            HAL_CAN_AddTxMessage(hcan,&TX_header,Txbuf,&TxMAailBox);

}




float Mode_Long(float a,float b){
    return sqrtf(a*a+b*b);
}

