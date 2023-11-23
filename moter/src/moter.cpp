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


void Send_Data_DM_PV(CAN_HandleTypeDef *hcan,uint16_t REAL_CAN_ID , float angle , float Velocity_Value)
{//position的取值范围是正负12.5，velocity的取值范围是正负45
//其中CAN的ID为配置电机的ID，后两项分别为位置的取值，以及速度的限制
//角度值进行换算,angle对应的是电机实际的角度值，
// “取值的范围是目前是20度到90度”
// position为对应的弧度值
    CAN_TxHeaderTypeDef TX_header;
    uint8_t Txbuf[8];
    uint32_t TxMAailBox;
    uint8_t* Position_buf;
    uint8_t* Velocity_buf;
    TX_header.DLC=0x08;

    TX_header.StdId=REAL_CAN_ID+0x100;
    TX_header.IDE=CAN_ID_STD;
    TX_header.RTR=CAN_RTR_DATA;


    float Position_Value;

    if(angle>90)angle = 90;
    if(angle<20)angle = 20;

    Position_Value = angle*PI/180.0;

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

void Send_Data_DM_V(CAN_HandleTypeDef *hcan,uint16_t REAL_CAN_ID , float Velocity_Value)
{//velocity的取值范围是正负45
//其中CAN的ID为配置电机的ID，后两项分别为位置的取值，以及速度的限制

    CAN_TxHeaderTypeDef TX_header;
    uint8_t Txbuf[4];
    uint32_t TxMAailBox;
    uint8_t* Position_buf;
    uint8_t* Velocity_buf;
    TX_header.DLC=0x04;

    TX_header.StdId=REAL_CAN_ID+0x200;
    TX_header.IDE=CAN_ID_STD;
    TX_header.RTR=CAN_RTR_DATA;

    Velocity_buf=(uint8_t*)&Velocity_Value;
    Txbuf[0]=*(Velocity_buf);
    Txbuf[1]=*(Velocity_buf+1);
    Txbuf[2]=*(Velocity_buf+2);
    Txbuf[3]=*(Velocity_buf+3);

    HAL_CAN_AddTxMessage(hcan,&TX_header,Txbuf,&TxMAailBox);

}

void Send_Data_DM_Mit(CAN_HandleTypeDef *hcan,uint16_t REAL_CAN_ID , float angle , float Velocity_Value,float KP_Value, float KD_Value, float Torque_Value)
{//position的取值范围是正负12.5(理论上)，velocity的取值范围是正负45(理论上)
//其中CAN的ID为配置电机的ID，后两项分别为位置的取值，以及速度的限制
//角度值进行换算,angle对应的是电机实际的角度值，
// “取值的范围是目前是20度到95度”
// position为对应的弧度值
    CAN_TxHeaderTypeDef TX_header;
    uint8_t Txbuf[8];
    uint32_t TxMAailBox;
    uint8_t* Position_buf;
    uint8_t* Velocity_buf;
    TX_header.DLC=0x08;

    TX_header.StdId=REAL_CAN_ID;
    TX_header.IDE=CAN_ID_STD;
    TX_header.RTR=CAN_RTR_DATA;

    float Position_Value;

    // if(angle>100)angle = 90;
    // if(angle<20)angle = 20;

    Position_Value = angle*PI/180.0;

    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    pos_tmp = float_to_uint(Position_Value, POSITION_MIN, POSITION_MAX, 16);
    vel_tmp = float_to_uint(Velocity_Value, VELOCITY_MIN, VELOCITY_MAX, 12);
    kp_tmp = float_to_uint(KP_Value, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(KD_Value, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(Torque_Value, TORQUE_MIN, TORQUE_MAX, 12);


    Txbuf[0] = (pos_tmp >> 8);
    Txbuf[1] = pos_tmp;
    Txbuf[2] = (vel_tmp >> 4);
    Txbuf[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    Txbuf[4] = kp_tmp;
    Txbuf[5] = (kd_tmp >> 4);
    Txbuf[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    Txbuf[7] = tor_tmp;
    // Position_buf=(uint8_t*)&Position_Value;
    // Velocity_buf=(uint8_t*)&Velocity_Value;
    // Txbuf[0]=*(Position_buf);
    // Txbuf[1]=*(Position_buf+1);
    // Txbuf[2]=*(Position_buf+2);
    // Txbuf[3]=*(Position_buf+3);
    // Txbuf[4]=*(Velocity_buf);
    // Txbuf[5]=*(Velocity_buf+1);
    // Txbuf[6]=*(Velocity_buf+2);
    // Txbuf[7]=*(Velocity_buf+3);

    HAL_CAN_AddTxMessage(hcan,&TX_header,Txbuf,&TxMAailBox);

}

void Send_Data_DM_Control(CAN_HandleTypeDef *hcan,uint16_t REAL_CAN_ID , int Mode , int Control_Mode)
{//mode为1时，将使能电机，mode为0时，将失能电机，mode为2时，将保存位置零点，mode为3时，将清除错误（例如过热）
//Control_Mode为0时，mit模式;为1时，位置速度模式，2:速度模式
    CAN_TxHeaderTypeDef TX_header;
    uint8_t Txbuf[8];
    uint32_t TxMAailBox;
    TX_header.DLC=8;

    switch (Control_Mode)
    {
        case 0://mit模式
            TX_header.StdId=REAL_CAN_ID;
            break;
        case 1://位置速度模式
            TX_header.StdId=REAL_CAN_ID+0x100;
            break;
        case 2://速度模式
            TX_header.StdId=REAL_CAN_ID+0x200;
            break;
        default:
            break;
    }
    TX_header.IDE=CAN_ID_STD;
    TX_header.RTR=CAN_RTR_DATA;

    int i;
    for (i=0;i<=6;i++)
        {
            Txbuf[i]=0xFF;
        }

    switch (Mode)
    {
        case 0://失能电机
            Txbuf[7]=0xFD;
            break;
        
        case 1://使能电机
            Txbuf[7]=0xFC;
            break;

        case 2://保存位置零点
            Txbuf[7]=0xFE;
            break;

        case 3://清除错误
            Txbuf[7]=0xFB;
            break;

        default:
            break;
    }

     HAL_CAN_AddTxMessage(hcan,&TX_header,Txbuf,&TxMAailBox);
}

float Mode_Long(float a,float b){
    return sqrtf(a*a+b*b);
}

