#include "main.h"
#include "wheel.h"




void wheel_driver::wheel_init(uint8_t ID){
        CAN_ID=ID;
        current_target=0;
        velocity_target=0;
        location_target=0;
}



void wheel_driver::Driver_Enable(){
        CAN_TxHeaderTypeDef TX_header;
        TX_header.DLC=5;
        TX_header.IDE=CAN_ID_EXT;
        TX_header.RTR=CAN_RTR_DATA;
        TX_header.ExtId=0x05010013|(((uint16_t)CAN_ID)<<8);

        uint8_t TX_buf[5];
        uint32_t TxMailbox;
        TX_buf[0]=0x05;
        TX_buf[1]=0x01;
        TX_buf[2]=CAN_ID;
        TX_buf[3]=0x01;
        TX_buf[4]=CAN_ID;

        HAL_CAN_AddTxMessage(hcan,&TX_header,TX_buf,&TxMailbox);

}

void wheel_driver::Velocity_Servo_Enable(){
        CAN_TxHeaderTypeDef TX_header;
        TX_header.DLC=3;
        TX_header.IDE=CAN_ID_EXT;
        TX_header.RTR=CAN_RTR_DATA;
        TX_header.ExtId=0x05010011|(((uint16_t)CAN_ID)<<8);

        uint8_t TX_buf[3];
        uint32_t TxMailbox;
        TX_buf[0]=0x00;
        TX_buf[1]=0xa;
        TX_buf[2]=0;


        HAL_CAN_AddTxMessage(hcan,&TX_header,TX_buf,&TxMailbox);  
}

void wheel_driver::Location_Servo_Enable(){
        CAN_TxHeaderTypeDef TX_header;
        TX_header.DLC=3;
        TX_header.IDE=CAN_ID_EXT;
        TX_header.RTR=CAN_RTR_DATA;
        TX_header.ExtId=0x05010011|(((uint16_t)CAN_ID)<<8);

        uint8_t TX_buf[3];
        uint32_t TxMailbox;
        TX_buf[0]=0x01;
        TX_buf[1]=0xa;
        TX_buf[2]=0;


        HAL_CAN_AddTxMessage(hcan,&TX_header,TX_buf,&TxMailbox); 
}

void wheel_driver::Current_Servo_Enable(){
        CAN_TxHeaderTypeDef TX_header;
        TX_header.DLC=3;
        TX_header.IDE=CAN_ID_EXT;
        TX_header.RTR=CAN_RTR_DATA;
        TX_header.ExtId=0x05010011|(((uint16_t)CAN_ID)<<8);

        uint8_t TX_buf[3];
        uint32_t TxMailbox;
        TX_buf[0]=0x02;
        TX_buf[1]=0xa;
        TX_buf[2]=0;


        HAL_CAN_AddTxMessage(hcan,&TX_header,TX_buf,&TxMailbox); 
}



void wheel_driver::send_velocity(){
        CAN_TxHeaderTypeDef TX_header;
        uint8_t TX_buf[4];
        uint32_t TxMailbox;

        TX_buf[0]=velocity_target&0xff;
        TX_buf[1]=(velocity_target>>8)&0xff;
        TX_buf[2]=(velocity_target>>16)&0xff;
        TX_buf[3]=(velocity_target>>24)&0xff;

        TX_header.DLC=4;
        TX_header.ExtId=0x05010013|(((uint16_t)CAN_ID)<<8);
        TX_header.IDE=CAN_ID_EXT;
        TX_header.RTR=CAN_RTR_DATA;

        HAL_CAN_AddTxMessage(hcan,&TX_header,TX_buf,&TxMailbox);

}

void wheel_driver::take_driver_parameter(){
        CAN_TxHeaderTypeDef TX_header;
        uint8_t TX_buf[2];
        uint32_t TxMailbox;
        TX_header.DLC=0;
        TX_header.ExtId=0x05010014|(((uint16_t)CAN_ID)<<8);
        TX_header.IDE=CAN_ID_EXT;
        TX_header.RTR=CAN_RTR_DATA;

        HAL_CAN_AddTxMessage(hcan,&TX_header,TX_buf,&TxMailbox); 

}


void wheel_driver::Can_Data_processing(uint32_t ID,uint8_t buf[]){




        uint16_t velocity_loc;
        int16_t acceleration_now;
        int16_t deceleration_now;
        uint16_t current_limit;
        

        switch (ID&0xffff00ff){
                case 0x050100B2:
                current_real=(buf[0])|(buf[1]<<8)|(buf[2]<<16)|(buf[3]<<24);
                temperature=(buf[4])|(buf[5]<<8);
                
               
                break;

                case 0x050100B3:
                location_real=((buf[0])|(buf[1]<<8)|(buf[2]<<16)|(buf[3]<<24))/1024.0f;
                velocity_real=buf[4]|(buf[5]<<8)|(buf[6]<<16)|(buf[7]<<24);
                break;
                
                case 0x050100B4:

                velocity_loc=(buf[0])|(buf[1]<<8);
                acceleration_now=(buf[2])|(buf[3]<<8);
                deceleration_now=(buf[4])|(buf[5]<<8);
                current_limit=(buf[6])|(buf[7]<<8);
                break;

        }



}



     





