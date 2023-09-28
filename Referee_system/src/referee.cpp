#include "referee.h"
#include "main.h"
#include "usart.h"
#include "chassis.h"
#include "remote.h"
#include "gimbal.h"
#include "math.h"

uint8_t UI_flag = 1;
uint8_t referee_buf[REFER_NUM_MAX];
Referee_System Referee;

uint8_t UI_Rx_buf[REFER_NUM_MAX];

extern Class_Remote_data Remote;
extern Class_Gimbal Gimbal;
extern Class_Super_Cup Super_Cup;
extern int32_t Shoot_speed_flag;
extern uint8_t gyro_state_flag_left;
extern uint8_t gyro_state_flag_right;
void Referee_System::Referee_system_init(UART_HandleTypeDef *HUART){
        huart=HUART;
}


void Referee_System::Delete_graphic_none(uint8_t graphic_id){
        uint8_t referee_tx_buf[8];
        uint16_t num_id=0x0100;
        uint16_t robot_id=Game_robot_status.robot_id;
        uint16_t client_id=robot_id+256;
        referee_buf[6]=0;
        referee_buf[7]=graphic_id;
        memcpy(referee_tx_buf,&num_id,2);
        memcpy(referee_tx_buf+2,&robot_id,2);
        memcpy(referee_tx_buf+4,&client_id,2);
}


void Referee_System::Delete_graphic(uint8_t graphic_id){
        uint8_t referee_tx_buf[8];
        uint16_t num_id=0x0100;
        uint16_t robot_id=Game_robot_status.robot_id;
        uint16_t client_id=robot_id+256;
        referee_buf[6]=1;
        referee_buf[7]=graphic_id;
        memcpy(referee_tx_buf,&num_id,2);
        memcpy(referee_tx_buf+2,&robot_id,2);
        memcpy(referee_tx_buf+4,&client_id,2);

}


void Referee_System::Delete_graphic_all(){
        uint8_t referee_tx_buf[8];
        uint16_t num_id=0x0100;
        uint16_t robot_id=Game_robot_status.robot_id;
        uint16_t client_id=robot_id+256;
        referee_buf[6]=2;
        memcpy(referee_tx_buf,&num_id,2);
        memcpy(referee_tx_buf+2,&robot_id,2);
        memcpy(referee_tx_buf+4,&client_id,2);
}

void Referee_System::Draw_line(graphic_data_struct_t *Graphic_data,uint8_t name[],uint8_t operate_id,uint8_t graphic_id,uint8_t color_id,uint16_t Width,uint16_t Start_x,uint16_t Start_y,uint16_t End_x,uint16_t End_y){
        memcpy(Graphic_data->graphic_name,name,3);
        Graphic_data->operate_tpye=operate_id;
        Graphic_data->graphic_tpye=0;
        Graphic_data->layer=graphic_id;
        Graphic_data->color=color_id;
        Graphic_data->width=Width;
        Graphic_data->start_x=Start_x;
        Graphic_data->start_y=Start_y;
        Graphic_data->end_x=End_x;
        Graphic_data->end_y=End_y;
}

void Referee_System::Draw_rectangle(graphic_data_struct_t *Graphic_data,uint8_t name[],uint8_t operate_id,uint8_t graphic_id,uint8_t color_id,uint16_t Width,uint16_t Start_x,uint16_t Start_y,uint16_t End_x,uint16_t End_y){
        memcpy(Graphic_data->graphic_name,name,3);
        Graphic_data->operate_tpye=operate_id;
        Graphic_data->graphic_tpye=1;
        Graphic_data->layer=graphic_id;
        Graphic_data->color=color_id;
        Graphic_data->width=Width;
        Graphic_data->start_x=Start_x;
        Graphic_data->start_y=Start_y;
        Graphic_data->end_x=End_x;
        Graphic_data->end_y=End_y;
}

/*对整圆进行操作*/
void Referee_System::Draw_circular(graphic_data_struct_t *Graphic_data,uint8_t name[],uint8_t operate_id,uint8_t graphic_id,uint8_t color_id,uint16_t Width,uint16_t Start_x,uint16_t Start_y,uint16_t Radius){
        memcpy(Graphic_data->graphic_name,name,3);
        Graphic_data->operate_tpye=operate_id;
        Graphic_data->graphic_tpye=2;
        Graphic_data->layer=graphic_id;
        Graphic_data->color=color_id;
        Graphic_data->width=Width;
        Graphic_data->start_x=Start_x;
        Graphic_data->start_y=Start_y;
        Graphic_data->radius=Radius;
}

void Referee_System::Draw_elliptical(graphic_data_struct_t *Graphic_data,uint8_t name[],uint8_t operate_id,uint8_t graphic_id,uint8_t color_id,uint16_t Width,uint16_t Start_x,uint16_t Start_y,uint16_t End_x,uint16_t End_y){
        memcpy(Graphic_data->graphic_name,name,3);
        Graphic_data->operate_tpye=operate_id;
        Graphic_data->graphic_tpye=3;
        Graphic_data->layer=graphic_id;
        Graphic_data->color=color_id;
        Graphic_data->width=Width;
        Graphic_data->start_x=Start_x;
        Graphic_data->start_y=Start_y;
        Graphic_data->end_x=End_x;
        Graphic_data->end_y=End_y;
}
void Referee_System::Draw_circular_arc(graphic_data_struct_t *Graphic_data,uint8_t name[],uint8_t operate_id,uint8_t graphic_id,uint8_t color_id,uint16_t Start_angle,uint16_t End_angle,uint16_t Width,uint16_t Start_x,uint16_t Start_y,uint16_t End_x,uint16_t End_y){
        memcpy(Graphic_data->graphic_name,name,3);
        Graphic_data->operate_tpye=operate_id;
        Graphic_data->graphic_tpye=4;
        Graphic_data->layer=graphic_id;
        Graphic_data->color=color_id;
        Graphic_data->start_angle=Start_angle;
        Graphic_data->end_angle=End_angle;
        Graphic_data->width=Width;
        Graphic_data->start_x=Start_x;
        Graphic_data->start_y=Start_y;
        Graphic_data->end_x=End_x;
        Graphic_data->end_y=End_y;
}
void Referee_System::Draw_float(graphic_data_struct_t *Graphic_data,uint8_t name[],uint8_t operate_id,uint8_t graphic_id,uint8_t color_id,uint16_t Text_size,uint16_t Effective_number,uint16_t Width,uint16_t Start_x,uint16_t Start_y,float FLOAT_value){
        int32_t temp;
		static uint8_t* drawing_ptr = (uint8_t* )Graphic_data;
        memcpy(Graphic_data->graphic_name,name,3);
        Graphic_data->operate_tpye=operate_id;
        Graphic_data->graphic_tpye=5;
        Graphic_data->layer=graphic_id;
        Graphic_data->color=color_id;
        Graphic_data->start_angle=Text_size;
        Graphic_data->end_angle=Effective_number;
        Graphic_data->width=Width;
        Graphic_data->start_x=Start_x;
        Graphic_data->start_y=Start_y;
        temp=(int32_t)(1000*FLOAT_value);
		Graphic_data->end_y = temp;
		memcpy((void*)(drawing_ptr + 11),(uint8_t *)&temp,4);

}

void Referee_System::Draw_int(graphic_data_struct_t *Graphic_data,uint8_t name[],uint8_t operate_id,uint8_t graphic_id,uint8_t color_id,uint16_t Text_size,uint16_t Width,uint16_t Start_x,uint16_t Start_y,int32_t INT_value){
        static uint8_t* Int_ptr = (uint8_t* )Graphic_data;
        memcpy(Graphic_data->graphic_name,name,3);
        Graphic_data->operate_tpye=operate_id;
        Graphic_data->graphic_tpye=6;
        Graphic_data->layer=graphic_id;
        Graphic_data->color=color_id;
        Graphic_data->start_angle=Text_size;
        Graphic_data->width=Width;
        Graphic_data->start_x=Start_x;
        Graphic_data->start_y=Start_y;
        memcpy((void*)(Int_ptr + 11),(uint8_t *)&INT_value,4); 
}

void Referee_System::Draw_char(ext_client_custom_character_t *Graphic_data,uint8_t name[],uint8_t operate_id,uint8_t graphic_id,uint8_t color_id,uint16_t Text_size,uint16_t CHAR_length,uint16_t Width,uint16_t Start_x,uint16_t Start_y,uint8_t *Char_date){
        
        memcpy(Graphic_data->grapic_data_struct.graphic_name,name,3);
        Graphic_data->grapic_data_struct.operate_tpye=operate_id;
        Graphic_data->grapic_data_struct.graphic_tpye=7;
        Graphic_data->grapic_data_struct.layer=graphic_id;
        Graphic_data->grapic_data_struct.color=color_id;
        Graphic_data->grapic_data_struct.start_angle=Text_size;
        Graphic_data->grapic_data_struct.end_angle=CHAR_length;
        Graphic_data->grapic_data_struct.width=Width;
        Graphic_data->grapic_data_struct.start_x=Start_x;
        Graphic_data->grapic_data_struct.start_y=Start_y;   
	    memcpy(Graphic_data->data,Char_date,CHAR_length);
	
}


void Referee_System::UART_Data_processing(uint8_t buf[],uint16_t size){
	uint8_t* buf_origin = buf;
	while((buf-buf_origin)<260){
		if(*buf == 0xA5){
			memcpy(&Referee.Frame_header,buf,5);
			memcpy(&Referee.cmd_id,buf+5,2);
			buf += 7;
			if(Referee.Frame_header.SOF==0xA5){

				switch (Referee.cmd_id)
				{
				case 0x0001:{
					memcpy(&Referee.Game_state,buf,Referee.Frame_header.data_length);
					break;
				}
				case 0x0002:{
					memcpy(&Referee.Game_result,buf,Referee.Frame_header.data_length);
					break;
				}
				case 0x0003:{
					memcpy(&Referee.Game_robot_HP,buf,Referee.Frame_header.data_length);
					break;
				}
				case 0x0004:{
					//坐等飞镖数据完善
					break;
				}
				case 0x0005:{
					memcpy(&Referee.ICRA_buff_debuff_zone_and_lurk_status,buf,Referee.Frame_header.data_length);
					break;
				}
				case 0x0101:{
					memcpy(&Referee.Event_data,buf,Referee.Frame_header.data_length);
					break;
				}
				case 0x0102:{
					memcpy(&Referee.Supply_projectile_action,buf,Referee.Frame_header.data_length);
					break;
				}
				case 0x0104:{
					memcpy(&Referee.Referee_warning,buf,Referee.Frame_header.data_length);
					break;
				}
				case 0x0105:{
					memcpy(&Referee.Dart_remaining_time,buf,Referee.Frame_header.data_length);
					break;            
				}
				case 0x0201:{
					memcpy(&Referee.Game_robot_status,buf,Referee.Frame_header.data_length);
					
					if(Last_mains_power_chassis_output==1&&Referee.Game_robot_status.mains_power_chassis_output==0){
							
					}
					if(Last_mains_power_gimbal_output==1&&Referee.Game_robot_status.mains_power_gimbal_output==0){

					}
					if(Last_mains_power_shooter_output==1&&Referee.Game_robot_status.mains_power_shooter_output){

					}


					Last_mains_power_chassis_output=Referee.Game_robot_status.mains_power_chassis_output;
					Last_mains_power_gimbal_output=Referee.Game_robot_status.mains_power_gimbal_output;
					Last_mains_power_shooter_output=Referee.Game_robot_status.mains_power_shooter_output;
					break;   
				}
				case 0x0202:{
					memcpy(&Referee.Power_heat_data,buf,Referee.Frame_header.data_length);
					break;           
				}
				case 0x0203:{
					memcpy(&Referee.Game_robot_pos,buf,Referee.Frame_header.data_length);
					break; 
				}
				case 0x0204:{
					memcpy(&Referee.Buff,buf,Referee.Frame_header.data_length);
					break;             
				}
				case 0x0205:{
					memcpy(&Referee.Aerial_robot_energy,buf,Referee.Frame_header.data_length);
					break;             
				}
				case 0x0206:{
					memcpy(&Referee.Robot_hurt,buf,Referee.Frame_header.data_length);
					break;
				}
				case 0x0207:{
					memcpy(&Referee.Shoot_data,buf,Referee.Frame_header.data_length);
					break;            
				}
				case 0x0208:{
					memcpy(&Referee.Bullet_remaining,buf,Referee.Frame_header.data_length);
					break;
				}
				case 0x0209:{
					memcpy(&Referee.Rfid_status,buf,Referee.Frame_header.data_length);
					break;            
				}
				case 0x020A:{
					memcpy(&Referee.Dart_client_cmd,buf,Referee.Frame_header.data_length);
					break;
				}
		/*------以下为机器人间交互数据------*/
				case 0x0301:{

					break;
				}
				
				default:
					break;
				}

			}
			buf += Referee.Frame_header.data_length + 2;
		}
		else  buf++;
	}
    
}


/*绘制准星并发送，图层9*/
graphic_data_struct_t UI_Target[7];
void Draw_Target(void)
{
	uint8_t name1[] = "091";//图片名称，用于标识更改
	uint8_t name2[] = "092";
	uint8_t name3[] = "093";
	uint8_t name4[] = "094";
	uint8_t name5[] = "095";
	uint8_t name6[] = "096";
	uint8_t name7[] = "097";
	Referee.Draw_line(&UI_Target[0],name1,UI_GRAPHIC_ADD,9,UI_COLOR_CYAN,1,960,240,960,520);//垂直线
	Referee.Draw_line(&UI_Target[1],name2,UI_GRAPHIC_ADD,9,UI_COLOR_CYAN,1,955,260,965,260);//标 9.3m
	Referee.Draw_line(&UI_Target[2],name3,UI_GRAPHIC_ADD,9,UI_COLOR_CYAN,1,945,375,975,375);//标 5m
	Referee.Draw_line(&UI_Target[3],name4,UI_GRAPHIC_ADD,9,UI_COLOR_CYAN,1,940,415,980,415);//标 3m
	Referee.Draw_line(&UI_Target[4],name5,UI_GRAPHIC_ADD,9,UI_COLOR_CYAN,1,900,430,1020,430);//标 1m
	Referee.Draw_line(&UI_Target[5],name6,UI_GRAPHIC_ADD,9,UI_COLOR_CYAN,1,830,330,1090,330);//标 近战打击
	Referee.Draw_circular(&UI_Target[6],name7,UI_GRAPHIC_ADD,9,UI_COLOR_PINK,3,960,540,20);//中心圆

	/*Referee.Draw_line(&UI_Target[0],name1,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,1000,240,1000,560);//垂直线
	// Referee.Draw_line(&UI_Target[1],name2,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,920,520,1080,520);//上击打线
	// Referee.Draw_line(&UI_Target[1],name3,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,840,480,1160,480);//中心水平线
	// Referee.Draw_line(&UI_Target[3],name4,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,920,440,1080,440);//枪管轴心线
	// Referee.Draw_line(&UI_Target[4],name5,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,940,360,1060,360);//下击打线
	Referee.Draw_line(&UI_Target[1],name2,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,995,360,1005,360);//标 10m
	Referee.Draw_line(&UI_Target[2],name3,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,987,440,1013,440);//标 5m
	Referee.Draw_line(&UI_Target[3],name4,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,980,455,1020,455);//标 3m
	Referee.Draw_line(&UI_Target[4],name5,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,940,430,1060,430);//标 1m
	Referee.Draw_line(&UI_Target[5],name6,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,865,330,1135,330);//标 近战打击
	// Referee.Draw_line(&UI_Target[5],name6,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,960,310,1040,310);//远距离狙杀线
	Referee.Draw_circular(&UI_Target[6],name7,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,1000,480,10);//中心圆*/
	
	/*Referee.Draw_line(&UI_Target[0],name1,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,1000,240,1000,560);//垂直线
	// Referee.Draw_line(&UI_Target[1],name2,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,920,520,1080,520);//上击打线
	// Referee.Draw_line(&UI_Target[1],name3,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,840,480,1160,480);//中心水平线
	// Referee.Draw_line(&UI_Target[3],name4,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,920,440,1080,440);//枪管轴心线
	// Referee.Draw_line(&UI_Target[4],name5,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,940,360,1060,360);//下击打线
	Referee.Draw_line(&UI_Target[1],name2,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,994,461,1006,461);//标 10m
	Referee.Draw_line(&UI_Target[2],name3,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,988,479,1012,479);//标 5m
	Referee.Draw_line(&UI_Target[3],name4,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,980,480,1020,480);//标 3m
	Referee.Draw_line(&UI_Target[4],name5,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,940,430,1060,430);//标 1m
	Referee.Draw_line(&UI_Target[5],name6,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,865,330,1135,330);//标 近战打击
	// Referee.Draw_line(&UI_Target[5],name6,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,960,310,1040,310);//远距离狙杀线
	Referee.Draw_circular(&UI_Target[6],name7,UI_GRAPHIC_ADD,9,UI_COLOR_WHITE,1,1000,480,10);//中心圆*/
	if(Referee.Game_robot_status.robot_id == 0)
    {
		UI_Draw_Date_processing(7,UI_Target,UI_CLIENT_INFANTRY_RED_1,UI_ROBOT_INFANTRY_RED_1,UI_Rx_buf);//红方1步兵传给其客户端
	}
	else{
		UI_Draw_Date_processing(7,UI_Target,Referee.Game_robot_status.robot_id+256,Referee.Game_robot_status.robot_id,UI_Rx_buf);
	}
}

/*绘制字符串提示符并发送，图层8*/
ext_client_custom_character_t UI_Char[7];

uint8_t prompt_buf[REFER_NUM_MAX];
uint8_t UI_gyro[] = "Gyro";
uint8_t UI_auto[] = "AUTO";
uint8_t UI_shoot_level[] = "shoot";
void Draw_gyro(void)
{
	uint8_t name1[] = "081";//图片名称，用于标识更改
	Referee.Draw_char(&UI_Char[0],name1,UI_GRAPHIC_ADD,8,UI_COLOR_YELLOW,24,sizeof(UI_gyro),3,60,780,UI_gyro);//小陀螺模式的状态

	if(Referee.Game_robot_status.robot_id == 0)
    {
		UI_Char_Date_processing(UI_Char[0],UI_CLIENT_INFANTRY_RED_1,UI_ROBOT_INFANTRY_RED_1,prompt_buf);
	}
	else{
		UI_Char_Date_processing(UI_Char[0],Referee.Game_robot_status.robot_id+256,Referee.Game_robot_status.robot_id,prompt_buf);
	}

	
}


void Draw_auto_shoot(void)
{
	uint8_t name2[] = "082";//图片名称，用于标识更改
	Referee.Draw_char(&UI_Char[1],name2,UI_GRAPHIC_ADD,8,UI_COLOR_YELLOW,24,sizeof(UI_auto),3,60,700,UI_auto);//自瞄状态

	if(Referee.Game_robot_status.robot_id == 0)
    {
		UI_Char_Date_processing(UI_Char[1],UI_CLIENT_INFANTRY_RED_1,UI_ROBOT_INFANTRY_RED_1,prompt_buf);
	}
	else{
		UI_Char_Date_processing(UI_Char[1],Referee.Game_robot_status.robot_id+256,Referee.Game_robot_status.robot_id,prompt_buf);
	}


	
}

void Draw_shoot_level(void)
{
	uint8_t name3[] = "083";
	Referee.Draw_char(&UI_Char[2],name3,UI_GRAPHIC_ADD,8,UI_COLOR_YELLOW,24,sizeof(UI_shoot_level),3,1700,780,UI_shoot_level);//自瞄状态
	if(Referee.Game_robot_status.robot_id == 0)
    {
		UI_Char_Date_processing(UI_Char[2],UI_CLIENT_INFANTRY_RED_1,UI_ROBOT_INFANTRY_RED_1,prompt_buf);
	}
	else{
		UI_Char_Date_processing(UI_Char[2],Referee.Game_robot_status.robot_id+256,Referee.Game_robot_status.robot_id,prompt_buf);
	}

	
	
}


/*绘制超级电容图标以及部分圆标，图层7*/
graphic_data_struct_t UI_Super_Cup[7];
void Draw_super_cup(void)
{
	uint8_t name1[] = "071";//图片名称，用于标识更改
	uint8_t name2[] = "072";
	uint8_t name3[] = "073";
	uint8_t name4[] = "074";
	uint8_t name5[] = "075";
	uint8_t name6[] = "076";
	uint8_t name7[] = "077";
	Referee.Draw_circular(&UI_Super_Cup[0],name1,UI_GRAPHIC_ADD,7,UI_COLOR_YELLOW,3,200,770,20);//GYRO的状态圆
	Referee.Draw_circular(&UI_Super_Cup[1],name2,UI_GRAPHIC_ADD,7,UI_COLOR_YELLOW,3,200,690,20);//AUTO后的状态圆
	Referee.Draw_rectangle(&UI_Super_Cup[2],name3,UI_GRAPHIC_ADD,7,UI_COLOR_YELLOW,3,60,865,260,835);//电池图标
	Referee.Draw_circular(&UI_Super_Cup[3],name4,UI_GRAPHIC_ADD,7,UI_COLOR_YELLOW,3,1650,770,20);//SHOOT前的状态圆
	Referee.Draw_rectangle(&UI_Super_Cup[4],name5,UI_GRAPHIC_ADD,7,UI_COLOR_YELLOW,3,1750,700,1850,640);//功率限制

	if(Referee.Game_robot_status.robot_id == 0)
    {
		UI_Draw_Date_processing(5,UI_Super_Cup,UI_CLIENT_INFANTRY_RED_1,UI_ROBOT_INFANTRY_RED_1,UI_Rx_buf);
	}
	else{
		UI_Draw_Date_processing(5,UI_Super_Cup,Referee.Game_robot_status.robot_id+256,Referee.Game_robot_status.robot_id,UI_Rx_buf);
	}	
}
/*绘制底盘位置图标以及停车线，图层5*/
graphic_data_struct_t UI_parking_line[7];
void Draw_parking_line(void)
{
	uint8_t name1[] = "051";//图片名称，用于标识更改
	uint8_t name2[] = "052";
	uint8_t name3[] = "053";
	uint8_t name4[] = "054";
	uint8_t name5[] = "055";
	uint8_t name6[] = "056";
	uint8_t name7[] = "057";

	Referee.Draw_line(&UI_parking_line[0],name1,UI_GRAPHIC_ADD,5,UI_COLOR_GREEN,5,960,90,960,150);//枪头位置
	Referee.Draw_line(&UI_parking_line[1],name2,UI_GRAPHIC_ADD,5,UI_COLOR_PURPLE,3,600,0,800,250);//左停车线（小）
	Referee.Draw_line(&UI_parking_line[2],name3,UI_GRAPHIC_ADD,5,UI_COLOR_PURPLE,3,500,0,740,250);//（大）
	Referee.Draw_line(&UI_parking_line[3],name4,UI_GRAPHIC_ADD,5,UI_COLOR_PURPLE,3,1320,0,1160,250);//右停车线（小）
	Referee.Draw_line(&UI_parking_line[4],name5,UI_GRAPHIC_ADD,5,UI_COLOR_PURPLE,3,1420,0,1220,250);//（大）
	Referee.Draw_rectangle(&UI_parking_line[5],name6,UI_GRAPHIC_ADD,5,UI_COLOR_WHITE,2,1220,330,700,800);//自瞄框
	if(Referee.Game_robot_status.robot_id == 0)
    {
		UI_Draw_Date_processing(7,UI_parking_line,UI_CLIENT_INFANTRY_RED_1,UI_ROBOT_INFANTRY_RED_1,UI_Rx_buf);
	}
	else{
		UI_Draw_Date_processing(7,UI_parking_line,Referee.Game_robot_status.robot_id+256,Referee.Game_robot_status.robot_id,UI_Rx_buf);
	}	
}

/*动态图标1 小陀螺，自瞄，超级电容，图层6*/
graphic_data_struct_t UI_dynamic_icon[7];
//graphic_data_struct_t UI_shoot_speed;
//graphic_data_struct_t UI_super_cup_power;
//graphic_data_struct_t UI_chassis_power_limit;
int32_t Proport_energy_remaining;
void Draw_dynamic_icon(void)
{
	static uint8_t UI_dynamic_icon_flag = 0;
	static uint16_t UI_power_proportion = 0;//超级电容电量比例
	UI_power_proportion = Proport_energy_remaining / 10;
	uint8_t name1[] = "061";//图片名称，用于标识更改
	uint8_t name2[] = "062";
	uint8_t name3[] = "063";
	uint8_t name4[] = "064";
	uint8_t name5[] = "065";
	uint8_t name6[] = "066";
	
	
	Proport_energy_remaining = Super_Cup.get_Energy_rest();
	
	if(UI_dynamic_icon_flag == 0)
	{
		if(Remote.Gyro_state == 0)//小陀螺状态
		{
			Referee.Draw_circular(&UI_dynamic_icon[0],name1,UI_GRAPHIC_ADD,6,UI_COLOR_WHITE,14,200,770,7);//GYRO的状态圆
		}
		else
		{
			if(gyro_state_flag_left == 0&&gyro_state_flag_right == 0)
			{
				Referee.Draw_circular(&UI_dynamic_icon[0],name1,UI_GRAPHIC_ADD,6,UI_COLOR_GREEN,14,200,770,7);//GYRO的状态圆
			}
			else
			{
				Referee.Draw_circular(&UI_dynamic_icon[0],name1,UI_GRAPHIC_ADD,6,UI_COLOR_PINK,14,200,770,7);//GYRO的状态圆
			}	
		}
		
		if(Gimbal.gimbal_auto == 0)//自瞄状态
		{
			Referee.Draw_circular(&UI_dynamic_icon[1],name2,UI_GRAPHIC_ADD,6,UI_COLOR_WHITE,14,200,690,7);//AUTO的状态圆
		}
		else
		{
			Referee.Draw_circular(&UI_dynamic_icon[1],name2,UI_GRAPHIC_ADD,6,UI_COLOR_GREEN,14,200,690,7);//AUTO的状态圆
		}
		


		
		if(Proport_energy_remaining < 500)//超级电容电量比例
		{
			Referee.Draw_int(&UI_dynamic_icon[2],name3,UI_GRAPHIC_ADD,6,UI_COLOR_PINK,12,2,70,900,Proport_energy_remaining);
			Referee.Draw_line(&UI_dynamic_icon[5],name6,UI_GRAPHIC_ADD,6,UI_COLOR_PINK,26,60,850,UI_power_proportion+60,850);
		}
		else
		{
			Referee.Draw_int(&UI_dynamic_icon[2],name3,UI_GRAPHIC_ADD,6,UI_COLOR_GREEN,12,2,70,900,Proport_energy_remaining);
			Referee.Draw_line(&UI_dynamic_icon[5],name6,UI_GRAPHIC_ADD,6,UI_COLOR_GREEN,26,60,850,UI_power_proportion+60,850);
		}
		Referee.Draw_circular(&UI_dynamic_icon[3],name4,UI_GRAPHIC_ADD,6,UI_COLOR_WHITE,14,1650,770,7);
		Referee.Draw_float(&UI_dynamic_icon[4],name5,UI_GRAPHIC_ADD,6,UI_COLOR_PINK,24,0,3,1770,680,(float)Referee.Game_robot_status.chassis_power_limit);//底盘功率限制
		
		
		
		UI_dynamic_icon_flag = 1;
	}
	else{
		if(Remote.Gyro_state == 0)//小陀螺状态
		{
			Referee.Draw_circular(&UI_dynamic_icon[0],name1,UI_GRAPHIC_MODIFY,6,UI_COLOR_WHITE,14,200,770,7);//GYRO的状态圆
		}
		else
		{
			if(gyro_state_flag_left == 0&&gyro_state_flag_right == 0)
			{
				Referee.Draw_circular(&UI_dynamic_icon[0],name1,UI_GRAPHIC_MODIFY,6,UI_COLOR_GREEN,14,200,770,7);//GYRO的状态圆
			}
			else
			{
				Referee.Draw_circular(&UI_dynamic_icon[0],name1,UI_GRAPHIC_MODIFY,6,UI_COLOR_PINK,14,200,770,7);//GYRO的状态圆
			}
		}
		
		if(Gimbal.gimbal_auto == 0)//自瞄状态
		{
			Referee.Draw_circular(&UI_dynamic_icon[1],name2,UI_GRAPHIC_MODIFY,6,UI_COLOR_WHITE,14,200,690,7);//GYRO的状态圆
		}
		else
		{
			Referee.Draw_circular(&UI_dynamic_icon[1],name2,UI_GRAPHIC_MODIFY,6,UI_COLOR_GREEN,14,200,690,7);//GYRO的状态圆
		}
		
		if(Proport_energy_remaining < 500)//超级电容电量比例
		{
			Referee.Draw_int(&UI_dynamic_icon[2],name3,UI_GRAPHIC_MODIFY,6,UI_COLOR_PINK,12,2,70,900,Proport_energy_remaining);
			Referee.Draw_line(&UI_dynamic_icon[5],name6,UI_GRAPHIC_MODIFY,6,UI_COLOR_PINK,26,60,850,UI_power_proportion+60,850);
		}
		else
		{
			Referee.Draw_int(&UI_dynamic_icon[2],name3,UI_GRAPHIC_MODIFY,6,UI_COLOR_GREEN,12,2,70,900,Proport_energy_remaining);
			Referee.Draw_line(&UI_dynamic_icon[5],name6,UI_GRAPHIC_MODIFY,6,UI_COLOR_GREEN,26,60,850,UI_power_proportion+60,850);
		}
		
		switch(Shoot_speed_flag)
		{
			case 1:
				Referee.Draw_circular(&UI_dynamic_icon[3],name4,UI_GRAPHIC_MODIFY,6,UI_COLOR_GREEN,14,1650,770,7);
				break;
			case 2:
				Referee.Draw_circular(&UI_dynamic_icon[3],name4,UI_GRAPHIC_MODIFY,6,UI_COLOR_YELLOW,14,1650,770,7);
				break;
			case 3:
				Referee.Draw_circular(&UI_dynamic_icon[3],name4,UI_GRAPHIC_MODIFY,6,UI_COLOR_PINK,14,1650,770,7);
				break;
			default:
				break;
		}
		Referee.Draw_float(&UI_dynamic_icon[4],name5,UI_GRAPHIC_MODIFY,6,UI_COLOR_PINK,24,0,3,1770,680,(float)Referee.Game_robot_status.chassis_power_limit);//底盘功率限制
		
	}
	if(Referee.Game_robot_status.robot_id == 0)
    {
		UI_Draw_Date_processing(7,UI_dynamic_icon,UI_CLIENT_INFANTRY_RED_1,UI_ROBOT_INFANTRY_RED_1,UI_Rx_buf);
	}
	else{
		UI_Draw_Date_processing(7,UI_dynamic_icon,Referee.Game_robot_status.robot_id+256,Referee.Game_robot_status.robot_id,UI_Rx_buf);
	}
}
 
/* 动态图标2 底盘位置，图层4*/
graphic_data_struct_t UI_dynamic_chassis[7];
void Draw_dynamic_chassis(void)
{
	static uint8_t UI_dynamic_chassis_flag = 0;
	//底盘四点坐标
	//右上为0，逆时针0123
	static uint16_t coordinate_x[4];
	static uint16_t coordinate_y[4];
	coordinate_x[0] = (uint16_t)(40 * cosf(PI/4.0f+(float)(Gimbal.yaw_real)/180.0f*PI)+960); 
	coordinate_x[1] = (uint16_t)(40 * cosf(3.0f*PI/4.0f+(float)(Gimbal.yaw_real)/180.0f*PI)+960); 
	coordinate_x[2] = (uint16_t)(40 * cosf(-3.0f*PI/4.0f+(float)(Gimbal.yaw_real)/180.0f*PI)+960); 
	coordinate_x[3] = (uint16_t)(40 * cosf(-PI/4.0f+(float)(Gimbal.yaw_real)/180.0f*PI)+960); 
	coordinate_y[0] = (uint16_t)(40 * sinf(PI/4.0f+(float)(Gimbal.yaw_real)/180.0f*PI)+90);
	coordinate_y[1] = (uint16_t)(40 * sinf(3.0f*PI/4.0f+(float)(Gimbal.yaw_real)/180.0f*PI)+90);
	coordinate_y[2] = (uint16_t)(40 * sinf(-3.0f*PI/4.0f+(float)(Gimbal.yaw_real)/180.0f*PI)+90);
	coordinate_y[3] = (uint16_t)(40 * sinf(-PI/4.0f+(float)(Gimbal.yaw_real)/180.0f*PI)+90);
	uint8_t name1[] = "041";//图片名称，用于标识更改
	uint8_t name2[] = "042";
	uint8_t name3[] = "043";
	uint8_t name4[] = "044";
	uint8_t name5[] = "045";
	uint8_t name6[] = "046";

	if(UI_dynamic_chassis_flag == 0)
	{
		Referee.Draw_line(&UI_dynamic_chassis[0],name1,UI_GRAPHIC_ADD,4,UI_COLOR_PINK,3,coordinate_x[0],coordinate_y[0],coordinate_x[1],coordinate_y[1]);
		Referee.Draw_line(&UI_dynamic_chassis[1],name2,UI_GRAPHIC_ADD,4,UI_COLOR_GREEN,3,coordinate_x[1],coordinate_y[1],coordinate_x[2],coordinate_y[2]);
		Referee.Draw_line(&UI_dynamic_chassis[2],name3,UI_GRAPHIC_ADD,4,UI_COLOR_GREEN,3,coordinate_x[2],coordinate_y[2],coordinate_x[3],coordinate_y[3]);
		Referee.Draw_line(&UI_dynamic_chassis[3],name4,UI_GRAPHIC_ADD,4,UI_COLOR_GREEN,3,coordinate_x[3],coordinate_y[3],coordinate_x[0],coordinate_y[0]);
		UI_dynamic_chassis_flag = 1;
	}
	else
	{
		Referee.Draw_line(&UI_dynamic_chassis[0],name1,UI_GRAPHIC_MODIFY,4,UI_COLOR_PINK,3,coordinate_x[0],coordinate_y[0],coordinate_x[1],coordinate_y[1]);
		Referee.Draw_line(&UI_dynamic_chassis[1],name2,UI_GRAPHIC_MODIFY,4,UI_COLOR_GREEN,3,coordinate_x[1],coordinate_y[1],coordinate_x[2],coordinate_y[2]);
		Referee.Draw_line(&UI_dynamic_chassis[2],name3,UI_GRAPHIC_MODIFY,4,UI_COLOR_GREEN,3,coordinate_x[2],coordinate_y[2],coordinate_x[3],coordinate_y[3]);
		Referee.Draw_line(&UI_dynamic_chassis[3],name4,UI_GRAPHIC_MODIFY,4,UI_COLOR_GREEN,3,coordinate_x[3],coordinate_y[3],coordinate_x[0],coordinate_y[0]);
	}
	if(Referee.Game_robot_status.robot_id == 0)
    {
		UI_Draw_Date_processing(5,UI_dynamic_chassis,UI_CLIENT_INFANTRY_RED_1,UI_ROBOT_INFANTRY_RED_1,UI_Rx_buf);
	}
	else{
		UI_Draw_Date_processing(5,UI_dynamic_chassis,Referee.Game_robot_status.robot_id+256,Referee.Game_robot_status.robot_id,UI_Rx_buf);
	}
}


/*推送图形，支持1，2，5，7*/
unsigned char UI_seq;

int UI_Draw_Date_processing(uint16_t num,graphic_data_struct_t Graphic_data[],uint16_t Cilent_ID,uint16_t Robot_ID,uint8_t UI_RX_buf[])
{
	uint16_t CMD_ID = 0x0301;
	unsigned char *frame_point;//读写指针
	uint16_t frametail = 0xFFFF;
	ext_student_interactive_header_data_t Interactive_Date;//交互数据接收信息
	
	frame_header UI_frame_head;
	frame_point = (unsigned char *)&UI_frame_head;
	UI_frame_head.SOF = 0xA5;
	UI_frame_head.seq = UI_seq;
	UI_frame_head.data_length = 6 + num * 15;
	UI_frame_head.CRC8 = Get_CRC8_Check_Sum_UI(frame_point,4,0xFF);
	memcpy(UI_RX_buf,&UI_frame_head,5);
	memcpy(UI_RX_buf+5,&CMD_ID,2);
	
	switch(num)//判断发送图像个数
	{
		case 1:
			Interactive_Date.data_cmd_id = UI_DATA_DRAW_1;
			break;
		case 2:
			Interactive_Date.data_cmd_id = UI_DATA_DRAW_2;
			break;
		case 5:
			Interactive_Date.data_cmd_id = UI_DATA_DRAW_5;
			break;
		case 7:
			Interactive_Date.data_cmd_id = UI_DATA_DRAW_7;
			break;
		default:
			return -1;
	}
	Interactive_Date.sender_ID = Robot_ID;
	Interactive_Date.receiver_ID = Cilent_ID;
	
	memcpy(UI_RX_buf+7,&Interactive_Date,6);
	memcpy(UI_RX_buf+13,Graphic_data,15 * num);
	
	frame_point = (unsigned char *)&UI_frame_head;
	frametail = Get_CRC16_Check_Sum_UI(frame_point,sizeof(UI_frame_head),frametail);
	
	frame_point = (unsigned char *)&CMD_ID;
	frametail = Get_CRC16_Check_Sum_UI(frame_point,sizeof(CMD_ID),frametail);
	
	frame_point = (unsigned char *)&Interactive_Date;
	frametail = Get_CRC16_Check_Sum_UI(frame_point,sizeof(Interactive_Date),frametail);
	
	frame_point = (unsigned char *)Graphic_data;
	frametail = Get_CRC16_Check_Sum_UI(frame_point,15 * num,frametail);
	
	memcpy(UI_RX_buf+13+15 * num,&frametail,2);
	
	HAL_UART_Transmit_IT(&huart6,UI_RX_buf,15+15*num);
	UI_seq++;
	return 0;
}

/*推送字符串*/

int UI_Char_Date_processing(ext_client_custom_character_t Graphic_data,uint16_t Cilent_ID,uint16_t Robot_ID,uint8_t UI_RX_buf[])
{
	uint16_t CMD_ID = 0x0301;
	unsigned char *frame_point;//读写指针
	uint16_t frametail = 0xFFFF;
	ext_student_interactive_header_data_t Interactive_Date;//交互数据接收信息
	
	frame_header UI_frame_head;
	frame_point = (unsigned char *)&UI_frame_head;
	UI_frame_head.SOF = 0xA5;
	UI_frame_head.seq = UI_seq;
	UI_frame_head.data_length = 6 + 45;
	UI_frame_head.CRC8 = Get_CRC8_Check_Sum_UI(frame_point,4,0xFF);
	memcpy(UI_RX_buf,&UI_frame_head,5);
	memcpy(UI_RX_buf+5,&CMD_ID,2);
	
	Interactive_Date.data_cmd_id = UI_DATA_DRAW_CHAR;
	Interactive_Date.sender_ID = Robot_ID;
	Interactive_Date.receiver_ID = Cilent_ID;
	
	memcpy(UI_RX_buf+7,&Interactive_Date,6);
	memcpy(UI_RX_buf+13,&Graphic_data,45);
	
	frame_point = (unsigned char *)&UI_frame_head;
	frametail = Get_CRC16_Check_Sum_UI(frame_point,sizeof(UI_frame_head),frametail);
	
	frame_point = (unsigned char *)&CMD_ID;
	frametail = Get_CRC16_Check_Sum_UI(frame_point,sizeof(CMD_ID),frametail);
	
	frame_point = (unsigned char *)&Interactive_Date;
	frametail = Get_CRC16_Check_Sum_UI(frame_point,sizeof(Interactive_Date),frametail);
	
	frame_point = (unsigned char *)&Graphic_data;
	frametail = Get_CRC16_Check_Sum_UI(frame_point,45,frametail);
	
	memcpy(UI_RX_buf+13+45,&frametail,2);
	
	HAL_UART_Transmit_IT(&huart6,UI_RX_buf,15+45);
	UI_seq++;
	return 0;
}









/*****************************************************CRC8校验值计算**********************************************/
const unsigned char CRC8_INIT_UI = 0xff; 
const unsigned char CRC8_TAB_UI[256] = 
{ 
0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41, 
0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc, 
0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62, 
0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff, 
0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07, 
0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a, 
0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24, 
0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9, 
0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd, 
0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50, 
0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee, 
0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73, 
0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b, 
0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16, 
0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8, 
0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35, 
};
unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8) 
{ 
	unsigned char ucIndex; 
	while (dwLength--) 
	{ 
		ucIndex = ucCRC8^(*pchMessage++); 
		ucCRC8 = CRC8_TAB_UI[ucIndex]; 
	} 
	return(ucCRC8); 
}

uint16_t CRC_INIT_UI = 0xffff; 
const uint16_t wCRC_Table_UI[256] = 
{ 
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 
0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7, 
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 
0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876, 
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 
0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5, 
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974, 
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3, 
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72, 
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1, 
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff, 
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e, 
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd, 
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c, 
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb, 
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a, 
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9, 
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
/* 
** Descriptions: CRC16 checksum function 
** Input: Data to check,Stream length, initialized checksum 
** Output: CRC checksum 
*/ 
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC) 
{ 
	uint8_t chData; 
	if (pchMessage == NULL) 
	{ 
		return 0xFFFF; 
	} 
	while(dwLength--) 
	{ 
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table_UI[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff]; 
	} 
	return wCRC; 
}


