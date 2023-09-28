#ifndef REFEREE_H
#define REFEREE_H



#ifdef __cplusplus       //-----------��׼д��-----------
extern "C"{                      //-----------��׼д��-----------
#endif

#include "main.h"
#include  "string.h"

#define REFER_NUM_MAX  300

	/*机器人ID*/	
	
#define UI_ROBOT_HERO_RED			1	//红方英雄
#define UI_ROBOT_ENGINEER_RED 		2	//红方工程
#define UI_ROBOT_INFANTRY_RED_1 	3	//红方步兵1
#define	UI_ROBOT_INFANTRY_RED_2		4	//红方步兵2
#define	UI_ROBOT_INFANTRY_RED_3		5	//红方步兵3
#define UI_ROBOT_FLY_RED			6	//红方空中支援
#define UI_ROBOT_SENTRY_RED			7	//红方哨兵
#define UI_ROBOT_RADAR_RED			9	//红方雷达站
	
#define UI_ROBOT_HERO_BLUE			101		//蓝方英雄
#define UI_ROBOT_ENGINEER_BLUE 		102		//蓝方工程
#define UI_ROBOT_INFANTRY_BLUE_1 	103		//蓝方步兵1
#define	UI_ROBOT_INFANTRY_BLUE_2	104		//蓝方步兵2
#define	UI_ROBOT_INFANTRY_BLUE_3	105		//蓝方步兵3
#define UI_ROBOT_FLY_BLUE			106		//蓝方空中支援
#define UI_ROBOT_SENTRY_BLUE		107		//红方哨兵
#define UI_ROBOT_RADAR_BLUE			109		//红方雷达站

	/*客户端ID*/
#define UI_CLIENT_HERO_RED			0X0101	//红方英雄操作手客户端
#define UI_CLIENT_ENGINEER_RED 		0X0102	//红方工程操作手客户端
#define UI_CLIENT_INFANTRY_RED_1 	0X0103	//红方步兵1操作手客户端
#define	UI_CLIENT_INFANTRY_RED_2	0X0104	//红方步兵2操作手客户端
#define	UI_CLIENT_INFANTRY_RED_3	0X0105	//红方步兵3操作手客户端
#define UI_CLIENT_FLY_RED			0X0106	//红方空中操作手客户端
	
#define UI_CLIENT_HERO_BLUE			0X0165	//蓝方英雄操作手客户端
#define UI_CLIENT_ENGINEER_BLUE 	0X0166	//蓝方工程操作手客户端
#define UI_CLIENT_INFANTRY_BLUE_1 	0X0167	//蓝方步兵1操作手客户端
#define	UI_CLIENT_INFANTRY_BLUE_2	0X0168	//蓝方步兵2操作手客户端
#define	UI_CLIENT_INFANTRY_BLUE_3	0X0169	//蓝方步兵3操作手客户端
#define UI_CLIENT_FLY_BLUE			0X016A	//蓝方空中操作手客户端
	
	/*图形操作*/
#define UI_GRAPHIC_NONE		0		//图形空操作
#define UI_GRAPHIC_ADD		1		//增加
#define UI_GRAPHIC_MODIFY	2		//修改
#define UI_GRAPHIC_DELETE	3		//删除

	/*图形类型*/
#define UI_GRAPHIC_LINE			0		//直线
#define UI_GRAPHIC_RECTANGLE	1		//矩形
#define UI_GRAPHIC_CIRCLE		2		//圆
#define UI_GRAPHIC_ELLIPSE		3		//椭圆
#define UI_GRAPHIC_ARC			4		//圆弧
#define UI_GRAPHIC_FLOAT		5		//浮点数
#define UI_GRAPHIC_INT			6		//整型数
#define UI_GRAPHIC_CHAR			7		//字符

	/*图形颜色*/
#define UI_COLOR_MAIN		0		//红蓝主色
#define UI_COLOR_YELLOW		1		//黄
#define UI_COLOR_GREEN		2		//绿
#define UI_COLOR_ORANGE		3		//橙
#define UI_COLOR_PURPLE		4		//紫
#define UI_COLOR_PINK		5		//粉
#define UI_COLOR_CYAN		6		//青
#define UI_COLOR_BLACK		7		//黑
#define UI_COLOR_WHITE		8		//白

/*交互数据接受信息ID*/
#define UI_DATA_EXCHANGE	0X0301

/*内容ID*/
#define UI_DATA_DELETE		0X100	//客户端删除图形
#define UI_DATA_DRAW_1		0X101	//客户端绘制一个图形
#define UI_DATA_DRAW_2		0X102	//客户端绘制二个图形
#define UI_DATA_DRAW_5		0X103	//客户端绘制五个图形
#define UI_DATA_DRAW_7		0X104	//客户端绘制七个图形
#define UI_DATA_DRAW_CHAR	0X110	//客户端绘制字符串

	
typedef __packed struct{
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
}frame_header;




/*------ 比赛状态数据：0x0001。发送频率：3Hz------*/
typedef __packed struct
{
uint8_t game_type : 4;
uint8_t game_progress : 4;
uint16_t stage_remain_time;
uint64_t SyncTimeStamp;
}ext_game_status_t;
/*------ 比赛结果数据：0x0002。发送频率：比赛结束后发送------*/
typedef __packed struct
{
 uint8_t winner;
}ext_game_result_t;

/*------  机器人血量数据：0x0003。发送频率：3Hz------*/
typedef __packed struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP; 
    uint16_t red_3_robot_HP; 
    uint16_t red_4_robot_HP; 
    uint16_t red_5_robot_HP; 
    uint16_t red_7_robot_HP; 
    uint16_t red_outpost_HP;
    uint16_t red_base_HP; 
    uint16_t blue_1_robot_HP; 
    uint16_t blue_2_robot_HP; 
    uint16_t blue_3_robot_HP; 
    uint16_t blue_4_robot_HP; 
    uint16_t blue_5_robot_HP; 
    uint16_t blue_7_robot_HP; 
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
}ext_game_robot_HP_t;



/*------  人工智能挑战赛加成\惩罚区分布与潜伏模式状态：0x0005。发送频率：1Hz 周期发送，发送范围：所有机器人------*/
typedef __packed struct
{
 uint8_t F1_zone_status:1;
 uint8_t F1_zone_buff_debuff_status:3; 
 uint8_t F2_zone_status:1;
 uint8_t F2_zone_buff_debuff_status:3; 
 uint8_t F3_zone_status:1;
 uint8_t F3_zone_buff_debuff_status:3; 
 uint8_t F4_zone_status:1;
 uint8_t F4_zone_buff_debuff_status:3; 
 uint8_t F5_zone_status:1;
 uint8_t F5_zone_buff_debuff_status:3; 
 uint8_t F6_zone_status:1;
 uint8_t F6_zone_buff_debuff_status:3;
 uint16_t red1_bullet_left;
 uint16_t red2_bullet_left;
uint16_t blue1_bullet_left;
uint16_t blue2_bullet_left;
uint8_t lurk_mode;
uint8_t res;
} ext_ICRA_buff_debuff_zone_and_lurk_status_t;



/*------  场地事件数据：0x0101。发送频率：3Hz------*/
typedef __packed struct
{
 uint32_t event_type;
} ext_event_data_t;

/*------  补给站动作标识：0x0102。发送频率：动作改变后发送, 发送范围：己方机器人------*/
typedef __packed struct
{
 uint8_t supply_projectile_id; 
 uint8_t supply_robot_id; 
 uint8_t supply_projectile_step; 
uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


/*------  裁判警告信息：0x0104。发送频率：警告发生后发送------*/
typedef __packed struct
{
 uint8_t level;
 uint8_t offending_robot_id;
}referee_warning_t;

/*------  飞镖发射口倒计时：cmd_id (0x0105)。发送频率：3Hz 周期发送，发送范围：己方机器人------*/
typedef __packed struct
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;





/*------  比赛机器人状态：0x0201。发送频率：10Hz------*/
typedef __packed struct
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_status_t;


/*------  实时功率热量数据：0x0202。发送频率：50Hz------*/
typedef __packed struct
{
 uint16_t chassis_volt; 
 uint16_t chassis_current; 
 float chassis_power; 
 uint16_t chassis_power_buffer; 
 uint16_t shooter_id1_17mm_cooling_heat;
uint16_t shooter_id2_17mm_cooling_heat;
uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;



/*------  机器人位置：0x0203。发送频率：10Hz------*/
typedef __packed struct
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

/*------  机器人增益：0x0204。发送频率：1Hz------*/
typedef __packed struct
{
    uint8_t power_rune_buff;
}ext_buff_t;


/*------  空中机器人能量状态：0x0205。发送频率：10Hz------*/
typedef __packed struct
{
    uint8_t attack_time;
} aerial_robot_energy_t;
/*------  伤害状态：0x0206。发送频率：伤害发生后发送------*/
typedef __packed struct
{
    uint8_t armor_id : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;


/*------  实时射击信息：0x0207。发送频率：射击后发送------*/
typedef __packed struct
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;
/*------  子弹剩余发射数：0x0208。发送频率：10Hz 周期发送，所有机器人发送------*/
typedef __packed struct
{
 uint16_t projectile_allowance_17mm;    //17mm 弹丸允许发弹量
 uint16_t projectile_allowance_42mm;    //42mm 弹丸允许发弹量
 uint16_t remaining_gold_coin;          //剩余金币数量
}projectile_allowance_t;

/*------   机器人 RFID 状态： 0x0209。发送频率： 3Hz，发送范围：单一机器人------*/
typedef __packed struct
{
uint32_t rfid_status;
} ext_rfid_status_t;

/*------   飞镖机器人客户端指令数据： 0x020A。发送频率： 10Hz，发送范围：单一机器人------*/
typedef __packed struct
{
uint8_t dart_launch_opening_status;
uint8_t dart_attack_target;
uint16_t target_change_time;
uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/* 地面机器人位置数据，对哨兵机器人发送:0x020B, 1Hz 频率发送*/
//场地围挡在红方补给站附近的交点为坐标原点，沿场地长边向蓝方为 X 轴正方向，沿场地短边向红方停机坪为 Y 轴正方向
typedef __packed struct
{
    float hero_x;               
    float hero_y;               
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float standard_5_x;
    float standard_5_y;
}ground_robot_position_t;

/* 雷达标记进度数据，向雷达发送: 0x020C ，以 1Hz 频率发送 */
typedef __packed struct
{
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
}radar_mark_data_t;





/*----------------------------------------------------------------*/
/*------以下为机器人间交互数据------*/
/*----------------------------------------------------------------*/


/*------   交互数据接收信息： 0x0301------*/
typedef __packed struct
{
    uint16_t data_cmd_id;                       // 数据段的内容 ID
    uint16_t sender_ID;                         //发送者的ID
    uint16_t receiver_ID;                       //接收者的ID
}ext_student_interactive_header_data_t;         

/* ID说明 */

/*
机器人 ID 编号如下所示：
1：红方英雄机器人
2：红方工程机器人
3/4/5：红方步兵机器人（与机器人 ID 3~5 对应）
6：红方空中机器人
7：红方哨兵机器人
8：红方飞镖
9：红方雷达
10：红方前哨站
11：红方基地

101：蓝方英雄机器人
102：蓝方工程机器人
103/104/105：蓝方步兵机器人（与机器人 ID 3~5 对应）
106：蓝方空中机器人
107：蓝方哨兵机器人
108：蓝方飞镖
109：蓝方雷达
110：蓝方前哨站
111：蓝方基地
*/

/*
选手端 ID 如下所示：
0x0101：红方英雄机器人选手端
0x0102：红方工程机器人选手端
0x0103/0x0104/0x0105：红方步兵机器人选手端（与机器人 ID3~5 对应）
0x0106：红方空中机器人选手端

0x0165：蓝方英雄机器人选手端
0x0166：蓝方工程机器人选手端
0x0167/0x0168/0x0169：蓝方步兵机器人选手端（与机器人 ID3~5 对应）
0x016A：蓝方空中机器人选手端
*/

/*客户端删除图层   0x0100  */
typedef __packed struct
{
uint8_t operate_tpye; 
uint8_t layer; 
} ext_client_custom_graphic_delete_t;



//图形数据,大小为15字节
typedef __packed struct
{ 
uint8_t graphic_name[3]; 
uint32_t operate_tpye:3; 
uint32_t graphic_tpye:3; 
uint32_t layer:4; 
uint32_t color:4; 
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10; 
uint32_t start_x:11; 
uint32_t start_y:11; 
uint32_t radius:10; 
uint32_t end_x:11; 
uint32_t end_y:11; 
} graphic_data_struct_t;

/*客户端绘制一个图形	数据的内容ID:0X0101*/
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;

/*客户端绘制两个图形	数据的内容ID:0X0102*/
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

/*客户端绘制五个图形	数据的内容ID:0X0103*/
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[5];
} ext_client_custom_graphic_five_t;

/*客户端绘制七个图形	数据的内容ID:0X0104*/
typedef __packed struct
{
	graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

/*客户端绘制字符	数据的内容ID:0X0110*/
typedef __packed struct
{	
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_custom_character_t;

/* 自定义控制器与机器人交互数据，发送方触发发送：0x302 频率上限为 30Hz */ 
typedef __packed struct
{
    uint8_t data[30];
}custom_robot_data_t;

/* 自定义控制器与选手端交互数据，发送方触发发送 : 0x306，频率上限为30Hz */ 
typedef __packed struct
{
    uint16_t key_value;
    uint16_t x_position:12;
    uint16_t mouse_left:4;
    uint16_t y_position:12;
    uint16_t mouse_right:4;
    uint16_t reserved;
}custom_client_data_t;

/* 选手端小地图接收哨兵数据 : 0x307，频率上限为 1Hz */ 
typedef __packed struct
{
 uint8_t intention;
 uint16_t start_position_x;
 uint16_t start_position_y;
 int8_t delta_x[49];
 int8_t delta_y[49];
}map_sentry_data_t;


class Referee_System
{
private:
    UART_HandleTypeDef *huart;
    uint8_t Last_mains_power_gimbal_output;
    uint8_t Last_mains_power_chassis_output;
    uint8_t Last_mains_power_shooter_output;

public:
    frame_header      Frame_header;
    uint16_t cmd_id;

    ext_game_status_t Game_state;       //比赛状态数据
    ext_game_result_t Game_result;      //比赛结果数据
    ext_game_robot_HP_t Game_robot_HP;  //比赛机器人血量数据
                                        //飞镖发射状态，坐等官方补全
    ext_ICRA_buff_debuff_zone_and_lurk_status_t ICRA_buff_debuff_zone_and_lurk_status;          //人工智能挑战赛加成与惩罚状态
    ext_event_data_t Event_data;        //场地时间数据
    ext_supply_projectile_action_t Supply_projectile_action;        //补给站动作表示
    referee_warning_t Referee_warning;      //裁判警告信息
    ext_dart_remaining_time_t Dart_remaining_time;      //飞镖发射口倒计时
    ext_game_robot_status_t Game_robot_status;      //比赛机器人状态
    ext_power_heat_data_t Power_heat_data;          //实施功率热量数据
    ext_game_robot_pos_t Game_robot_pos;        //机器人位置
    ext_buff_t  Buff;       //机器人增益
    aerial_robot_energy_t   Aerial_robot_energy;    //空中机器人能量状态
    ext_robot_hurt_t    Robot_hurt;         //伤害状态
    ext_shoot_data_t    Shoot_data;         //实时射击信息
    projectile_allowance_t  Bullet_remaining;       //子弹剩余发射数和兑换数 
    ext_rfid_status_t   Rfid_status;        //机器人 RFID 状态
    ext_dart_client_cmd_t   Dart_client_cmd;        //飞镖机器人客户端指令数据 
    ground_robot_position_t Ground_robot_position;  //地面机器人位置数据，对哨兵机器人发送
    radar_mark_data_t Radar_mark_data;      //雷达标记进度数据，向雷达发送

/*------以下为机器人间交互数据------*/
    ext_student_interactive_header_data_t   Student_interactive_header_data;        //交互数据接受信息
    custom_robot_data_t Custom_robot_data; //自定义控制器与机器人交互数据，发送方触发发送
    custom_client_data_t Custom_client_data;    //自定义控制器与选手端交互数据，发送方触发发送
    map_sentry_data_t Map_sentry_data;  //选手端小地图接收哨兵数据



    void Referee_system_init(UART_HandleTypeDef *HUART);

    void Delete_graphic_none(uint8_t graphic_id);//表示进行空操作
    void Delete_graphic(uint8_t graphic_id);//删除一个图层
    void Delete_graphic_all();//删除所有图层

    void Draw_line(graphic_data_struct_t *Graphic_data,uint8_t name[],uint8_t operate_id,uint8_t graphic_id,uint8_t color_id,uint16_t Width,uint16_t Start_x,uint16_t Start_y,uint16_t End_x,uint16_t End_y);
    void Draw_rectangle(graphic_data_struct_t *Graphic_data,uint8_t name[],uint8_t operate_id,uint8_t graphic_id,uint8_t color_id,uint16_t Width,uint16_t Start_x,uint16_t Start_y,uint16_t End_x,uint16_t End_y);
   void Draw_circular(graphic_data_struct_t *Graphic_data,uint8_t name[],uint8_t operate_id,uint8_t graphic_id,uint8_t color_id,uint16_t Width,uint16_t Start_x,uint16_t Start_y,uint16_t Radius);
    void Draw_elliptical(graphic_data_struct_t *Graphic_data,uint8_t name[],uint8_t operate_id,uint8_t graphic_id,uint8_t color_id,uint16_t Width,uint16_t Start_x,uint16_t Start_y,uint16_t End_x,uint16_t End_y);
    void Draw_circular_arc(graphic_data_struct_t *Graphic_data,uint8_t name[],uint8_t operate_id,uint8_t graphic_id,uint8_t color_id,uint16_t Start_angle,uint16_t End_angle,uint16_t Width,uint16_t Start_x,uint16_t Start_y,uint16_t End_x,uint16_t End_y);
    void Draw_float(graphic_data_struct_t *Graphic_data,uint8_t name[],uint8_t operate_id,uint8_t graphic_id,uint8_t color_id,uint16_t Text_size,uint16_t Effective_number,uint16_t Width,uint16_t Start_x,uint16_t Start_y,float FLOAT_value);
    void Draw_int(graphic_data_struct_t *Graphic_data,uint8_t name[],uint8_t operate_id,uint8_t graphic_id,uint8_t color_id,uint16_t Text_size,uint16_t Width,uint16_t Start_x,uint16_t Start_y,int32_t INT_value);
    void Draw_char(ext_client_custom_character_t *Graphic_data,uint8_t name[],uint8_t operate_id,uint8_t graphic_id,uint8_t color_id,uint16_t Text_size,uint16_t CHAR_length,uint16_t Width,uint16_t Start_x,uint16_t Start_y,uint8_t *Char_date);
    void UART_Data_processing(uint8_t buf[],uint16_t size);

};









void Draw_Target(void);//绘制准星
void Draw_gyro(void);
void Draw_auto_shoot(void);
void Draw_shoot_level(void);
void Draw_super_cup(void);
void Draw_dynamic_icon(void);
void Draw_parking_line(void);
void Draw_dynamic_chassis(void);

int UI_Draw_Date_processing(uint16_t num,graphic_data_struct_t Graphic_data[],uint16_t Cilent_ID,uint16_t Robot_ID,uint8_t UI_RX_buf[]);
int UI_Char_Date_processing(ext_client_custom_character_t Graphic_data,uint16_t Cilent_ID,uint16_t Robot_ID,uint8_t UI_RX_buf[]);

unsigned char Get_CRC8_Check_Sum_UI(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC) ;


#ifdef __cplusplus      //-----------��׼д��-----------
}                                          //-----------��׼д��-----------
#endif 




#endif
