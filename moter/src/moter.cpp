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


void Send_Data_DM(CAN_HandleTypeDef *hcan,uint16_t REAL_CAN_ID , float Position_Value , float Velocity_Value)
{//position的取值范围是正负12.5，velocity的取值范围是正负45
//其中CAN的ID为配置电机的ID，后两项分别为位置的取值，以及速度的限制
    CAN_TxHeaderTypeDef TX_header;
    uint8_t Txbuf[8];
    uint32_t TxMAailBox;
    uint8_t* Position_buf;
    uint8_t* Velocity_buf;
    TX_header.DLC=8;

    TX_header.StdId=REAL_CAN_ID+0x100;
    TX_header.IDE=CAN_ID_STD;
    TX_header.RTR=CAN_RTR_DATA;

    Position_Value = float_to_uint(Position_Value,POSITION_MIN,POSITION_MAX,16);
    Velocity_Value = float_to_uint(Velocity_Value,POSITION_MIN,POSITION_MAX,16);

    Position_buf=(uint8_t*)&Position_Value;
    Velocity_buf=(uint8_t*)&Velocity_Value;
    Txbuf[0]=*(Position_buf);
    Txbuf[1]=*(Position_buf+1);
    Txbuf[2]=*(Position_buf+2);
    Txbuf[3]=*(Position_buf+3);
    Txbuf[4]=*(Velocity_buf);
    Txbuf[5]=*(Velocity_buf+1);
    Txbuf[6]=*(Velocity_buf+2);
    Txbuf[7]=*(Velocity_buf+3);

    HAL_CAN_AddTxMessage(hcan,&TX_header,Txbuf,&TxMAailBox);

}

void Send_Data_DM_Contral(CAN_HandleTypeDef *hcan,uint16_t REAL_CAN_ID , int mode)
{//mode为1时，将使能电机，mode为0时，将失能电机，mode为2时，将保存位置零点，mode为3时，将清除错误（例如过热）
    CAN_TxHeaderTypeDef TX_header;
    uint8_t Txbuf[8];
    uint32_t TxMAailBox;
    TX_header.DLC=8;
    TX_header.StdId=REAL_CAN_ID+0x100;
    TX_header.IDE=CAN_ID_STD;
    TX_header.RTR=CAN_RTR_DATA;

    int i;
    for (i=0;i<=6;i++)
        {
            Txbuf[i]=0xFF;
        }

    switch (mode)
    {
    case 0://失能电机
        Txbuf[7]=0xFD;
        break;
    
    case 1://失能电机
        Txbuf[7]=0xFC;
        break;

    case 2://保存位置零点
        Txbuf[7]=0xFE;
        break;

    case 3://清除错误
        Txbuf[7]=0xFE;
        break;

    default:
        break;
    }

     HAL_CAN_AddTxMessage(hcan,&TX_header,Txbuf,&TxMAailBox);
}

float Mode_Long(float a,float b){
    return sqrtf(a*a+b*b);
}

