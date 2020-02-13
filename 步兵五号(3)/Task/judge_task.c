/**************************************************
* @file      judge_task.c
* @author    sjq
* @version   version 1.0
* @date      2019.3.31
* @brief     
***************************************************
* @attention

***************************************************
*/

/* Include ---------------------------------------*/
#include "judge_task.h"


/* Global Variable 比赛状态和步兵状态 -----------------------------*/
uint8_t gameprograss;					//当前比赛阶段
uint16_t stageremaintime;				//当前阶段剩余时间

uint8_t my_robot_id;
uint8_t robotlevel;						//机器人等级
uint8_t leftUpgratePoint;				//剩余性能点数

/** 防御 **/
uint8_t bloodlevel;						//血量性能等级
uint16_t remainHP;						//机器人剩余血量
uint16_t maxHP;							//机器人上限血量
uint8_t armorid;						//收到伤害的装甲片
uint8_t hurttype;						//血量变化类型


/** 机动 **/
uint8_t speedlevel;
uint16_t chassisvolt;					//底盘输出电压 mV
uint16_t chassiscurrent;				//底盘输出电流 mA
float chassispower;						//底盘输出功率 W
uint16_t chassispowerbuffer;			//底盘功率缓冲 J


/** 攻击 **/
uint8_t powerlevel;
uint16_t shooterheat0coolingrate;		//17mm枪口每秒冷却值
uint16_t shooterheat0coolinglimt;		//17mm枪口每秒热量上限
uint16_t shooterheat1coolingrate;		//42mm枪口每秒冷却值
uint16_t shooterheat1coolinglimt;		//42mm枪口每秒热量上限
uint16_t shooterheat0;					//17mm枪口热量
uint16_t shooterheat1;					//42mm枪口热量
uint8_t	bullettype;						//子弹类型
uint8_t	bulletfreq;						//子弹射频 Hz
float	bulletspeed;					//子弹射速 m/s




uint32_t  judge_tick = 0;
/****数据包结构定义****/
//cmd_id 命令码ID说明
//0001**********比赛状态数据: 0x0001     1Hz
typedef __packed struct
{
	uint8_t game_type : 4;					//比赛类型
	uint8_t game_progress : 4;			//当前比赛阶段
	uint16_t stage_remain_time;			//当前阶段剩余时间,s
}ext_game_state_t;	

//0006**********比赛结果数据: 0x0002     比赛结束后发送
typedef __packed struct
{
	uint8_t winner;									//0:平局,1:红方胜利,2:蓝方胜利
}ext_game_result_t;

//机器人存活数据: 0x0003    1Hz
typedef __packed struct
{
	uint16_t robot_legion;					//对应位置1表示存活,置0表示机器人死亡或未上场
}ext_game_robot_survivors_t;

//场地事件数据: 0x0101      1Hz
typedef __packed struct
{
	uint32_t event_type;
}ext_event_data_t;

//补给站动作标志: 0x0102    动作改变后发送
typedef __packed struct
{
	uint8_t supply_projectile_id;		//补给站口ID
	uint8_t supply_robot_id;				//补弹机器人ID
	uint8_t supply_projectile_step; //出弹口开闭状态
	uint8_t supply_projectile_num;	//补弹数量
}ext_supply_projectile_action_t;

//请求补给站补弹: 0x0103		上限10Hz,RM对抗赛尚未开放
typedef __packed struct
{
	uint8_t supply_projectile_id;		//补给站口ID
	uint8_t supply_robot_id;				//补弹机器人ID
	uint8_t supply_num;							//补弹数量
}ext_supply_projectile_booking_t;

//0001**********比赛机器人状态: 0x0201		   10Hz
typedef __packed struct
{
	uint8_t robot_id;								//机器人ID
	uint8_t robot_level;							//机器人等级
	uint16_t remain_HP;								//机器人剩余血量
	uint16_t max_HP;								//机器人上限血量
	uint16_t shooter_heat0_cooling_rate;			//机器人17mm枪口每秒冷却值
	uint16_t shooter_heat0_cooling_limit;		//机器人17mm枪口热量上限
	uint16_t shooter_heat1_cooling_rate;    	//机器人42mm枪口每秒冷却值
	uint16_t shooter_heat1_cooling_limit;		//机器人42mm枪口热量上限
	uint8_t mains_power_gimbal_output : 1;	//gimbal输出 1为24V 0为无
	uint8_t mains_power_chassis_output : 1;	//chassis输出 1为24V 0为无
	uint8_t mains_power_shooter_output : 1;	//shooter输出 1为24V 0为无
}ext_game_robot_state_t;

//0004**********实时功率热量数据: 0x0202		50Hz
typedef __packed struct
{
	uint16_t chassis_volt;					//底盘输出电压  mV
	uint16_t chassis_current;				//底盘输出电流  mA
	float chassis_power;					//底盘输出功率  W
	uint16_t chassis_power_buffer;			//底盘功率缓冲	J
	uint16_t shooter_heat0;					//17mm  枪口热量
	uint16_t shooter_heat1;					//42mm  枪口热量
}ext_power_heat_data_t;

//0008**********机器人位置: 0x0203		10Hz
typedef __packed struct
{
	float x;												//位置x坐标  m
	float y;												//位置y坐标  m
	float z;												//位置z坐标  m
	float yaw;											//位置枪口   度
}ext_game_robot_pos_t;

//机器人增益: 0x0204		状态改变后发送
typedef __packed struct
{
	uint8_t power_rune_buff;				//bit0: 机器人血量补血状态		bit1: 枪口热量冷却加速		bit2: 机器人防御加成		bit3: 机器人攻击加成
}ext_buff_musk_t;

//空中机器人能量状态: 0x0205		10Hz
typedef __packed struct
{
	uint8_t energy_point;						//积累的能量点
	uint8_t attack_time;						//可攻击时间  s     50s递减至0
}aerial_robot_energy_t;

//0002**********伤害状态: 0x0206		伤害发生后发送
typedef __packed struct
{
	uint8_t armor_id : 4;						//显示受到伤害的装甲板ID
	uint8_t hurt_type : 4;					//血量变化类型
}ext_robot_hurt_t;

//0003**********实时射击信息: 0x0207		射击后发送
typedef __packed struct
{
	uint8_t bullet_type;						//子弹类型
	uint8_t bullet_freq;						//子弹射频	Hz
	float bullet_speed;							//子弹射速	m/s
}ext_shoot_data_t;

//交互数据接受信息: 0x0301		上限10Hz
typedef __packed struct
{
	uint16_t data_cmd_id;						//数据段内容ID
	uint16_t send_ID;								//发送者的ID
	uint16_t receiver_ID;						//接受者的ID
	
	//可设置内容数据段		最大长度为113个字节
}ext_student_interactive_header_data_t;

//客户端自定义数据: 0x0301		内容ID: 0xD180		上限10Hz
typedef __packed struct
{
	float data1;
	float data2;
	float data3;
	uint8_t masks;
}client_custom_data_t;


//CRC校验码		crc8 generator polynomial:G(x)=x8+x5+x4+1
const unsigned char CRC8_INIT = 0xff;
const unsigned char CRC8_TAB[256] = {  
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

unsigned  char  Get_CRC8_Check_Sum(unsigned char  *pchMessage,unsigned int dwLength,unsigned char ucCRC8) { 
	unsigned char ucIndex;
	while (dwLength--) 
	{ 
		ucIndex = ucCRC8^(*pchMessage++); 
		ucCRC8   = CRC8_TAB[ucIndex]; 
	} 
	return(ucCRC8); 
}
/*
**   Descriptions:  CRC8 Verify function                                  
**   Input:         Data to Verify,Stream length = Data + checksum                    
**   Output:        True or False (CRC Verify Result)    
*/                  
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) {
	unsigned char ucExpected = 0; 
	if ((pchMessage == 0) || (dwLength <= 2)) return 0; 
	ucExpected = Get_CRC8_Check_Sum (pchMessage, dwLength-1, CRC8_INIT); 
	return (ucExpected == pchMessage[dwLength-1]);
}  

/*
**   Descriptions:  append CRC8 to the end of data                                
**   Input:         Data to CRC and append,Stream length = Data + checksum                    
**   Output:        True or False (CRC Verify Result)                                                      
*/ 
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) {  
	unsigned char ucCRC = 0; 
	if ((pchMessage == 0) || (dwLength <= 2)) return;
	ucCRC = Get_CRC8_Check_Sum ((unsigned char *)pchMessage, dwLength-1, CRC8_INIT); 
	pchMessage[dwLength-1] = ucCRC; 
}

uint16_t CRC_INIT = 0xffff;
const    uint16_t    wCRC_Table[256] = { 
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
**   Descriptions:  CRC16 checksum function                                  
**   Input:         Data to check,Stream length, initialized checksum                     
**   Output:        CRC checksum                                                          
*/ 
uint16_t    Get_CRC16_Check_Sum(uint8_t    *pchMessage,uint32_t    dwLength,uint16_t wCRC) {  
	uint8_t    chData; 
	if (pchMessage == NULL) 
	{ 
	return 0xFFFF; 
	}
	while(dwLength--) 
	{ 
	chData = *pchMessage++;
	(wCRC)  =  ((uint16_t)(wCRC)  >>  8)   ^  wCRC_Table[((uint16_t)(wCRC)  ^ 
	(uint16_t)(chData)) & 0x00ff];
	} 
	return wCRC;   
}

/*
**   Descriptions:  CRC16 Verify function                                  
**   Input:         Data to Verify,Stream length = Data + checksum                    
**   Output:        True or False (CRC Verify Result)                                                  
*/                  
uint32_t Verify_CRC16_Check_Sum(uint8_t    *pchMessage, uint32_t    dwLength) { 
	uint16_t wExpected = 0; 
	if ((pchMessage == NULL) || (dwLength <= 2)) 
	{ 
	return 0; // warn 
	} 
	wExpected = Get_CRC16_Check_Sum (pchMessage, dwLength - 2, CRC_INIT); 
	return ((wExpected &  0xff) == pchMessage[dwLength -  2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]); 
}

/*
**   Descriptions: append CRC16 to the end of data                                
**   Input:         Data to CRC and append,Stream length = Data + checksum                    
**   Output:        True or False (CRC Verify Result)                                                  
*/ 
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength) {  
		uint16_t wCRC = 0; 
	if ((pchMessage == NULL) || (dwLength <= 2)) 
	{ 
	return; 
	} 
	wCRC = Get_CRC16_Check_Sum ((uint8_t *)pchMessage, dwLength-2, CRC_INIT);
	pchMessage[dwLength-2] = (uint8_t)(wCRC & 0x00ff);
	pchMessage[dwLength-1] = (uint8_t)((wCRC >> 8)& 0x00ff);
}

/****数据包结构体定义*****/
typedef __packed struct { //0001 Frame1 比赛状态包
	U8 SOF;
	U16 data_length;
	U8 seq;
	U8 CRC8;
	U16 cmd_id;
	ext_game_state_t extGameState;
	U16 frame_tail;
} Frame1;

typedef __packed struct { //0002 Frame2 比赛结果数据包
	U8 SOF;
	U16 data_length;
	U8 seq;
	U8 CRC8;
	U16 cmd_id;
	ext_game_result_t extGameResult;
	U16 frame_tail;
} Frame2;

typedef __packed struct { //0003 Frame3 机器人存活数据包
	U8 SOF;
	U16 data_length;
	U8 seq;
	U8 CRC8;
	U16 cmd_id;
	ext_game_robot_survivors_t extGameRobotSurvivors;
	U16 frame_tail;
} Frame3;

typedef __packed struct { //0004 Frame4 场地事件数据包
	U8 SOF;
	U16 data_length;
	U8 seq;
	U8 CRC8;
	U16 cmd_id;
	ext_event_data_t extEventData;
	U16 frame_tail;
} Frame4;

typedef __packed struct { //0005 Frame5 补给站动作标志包
	U8 SOF;
	U16 data_length;
	U8 seq;
	U8 CRC8;
	U16 cmd_id;
	ext_supply_projectile_action_t extSupplyProjectileAction;
	U16 frame_tail;
} Frame5;

typedef __packed struct { //0006 Frame6 请求补给站补弹包
	U8 SOF;
	U16 data_length;
	U8 seq;
	U8 CRC8;
	U16 cmd_id;
	ext_supply_projectile_booking_t extSupplyProjectileBooking;
	U16 frame_tail;
} Frame6;

typedef __packed struct { //0007 Frame7 比赛机器人状态包
	U8 SOF;
	U16 data_length;
	U8 seq;
	U8 CRC8;
	U16 cmd_id;
	ext_game_robot_state_t extGameRobotState;
	U16 frame_tail;
} Frame7;

typedef __packed struct { //0008 Frame8 实时功率热量数据包
	U8 SOF;
	U16 data_length;
	U8 seq;
	U8 CRC8;
	U16 cmd_id;
	ext_power_heat_data_t extPowerHeatData;
	U16 frame_tail;
} Frame8;

typedef __packed struct { //0009 Frame9 机器人位置包
	U8 SOF;
	U16 data_length;
	U8 seq;
	U8 CRC8;
	U16 cmd_id;
	ext_game_robot_pos_t extGameRobotPos;
	U16 frame_tail;
} Frame9;

typedef __packed struct { //0010 Frame10 机器人增益包
	U8 SOF;
	U16 data_length;
	U8 seq;
	U8 CRC8;
	U16 cmd_id;
	ext_buff_musk_t extBuffMusk;
	U16 frame_tail;
} Frame10;

typedef __packed struct { //0011 Frame11 空中机器人能量状态包
	U8 SOF;
	U16 data_length;
	U8 seq;
	U8 CRC8;
	U16 cmd_id;
	aerial_robot_energy_t aerialRobotEnergy;
	U16 frame_tail;
} Frame11;

typedef __packed struct { //0012 Frame12 伤害状态包
	U8 SOF;
	U16 data_length;
	U8 seq;
	U8 CRC8;
	U16 cmd_id;
	ext_robot_hurt_t extRobotHurt;
	U16 frame_tail;
} Frame12;

typedef __packed struct { //0013 Frame13 实时射击信息包
	U8 SOF;
	U16 data_length;
	U8 seq;
	U8 CRC8;
	U16 cmd_id;
	ext_shoot_data_t extShootData;
	U16 frame_tail;
} Frame13;

typedef __packed struct { //0014 Frame14 交互数据接受信息包
	U8 SOF;
	U16 data_length;
	U8 seq;
	U8 CRC8;
	U16 cmd_id;
	ext_student_interactive_header_data_t extStudentInteractiveHeaderData;
	U16 frame_tail;
} Frame14;

typedef __packed struct { //0015 Frame15 客户端自定义数据包
	U8 SOF;
	U16 data_length;
	U8 seq;
	U8 CRC8;
	U16 cmd_id;
	ext_student_interactive_header_data_t extStudentInteractiveHeaderData;
	client_custom_data_t clientCustomData;
	U16 frame_tail;
} Frame15;


//Part: Student_interface
uint8_t USART_STU_BUF[200];     //学生接口接收缓冲,最大200个字节.
uint16_t USART_STU_STA=0;       //学生接口接收状态标记		
uint8_t USART_FRAME[46];
uint8_t USART_LEN = 0;
uint8_t recLength = 0;

static U8 TxBuf1[256];
volatile uint8_t TxCounter1=0,Txcursor1=0;
static uint32_t isrflags;
static uint32_t cr1its;


//////////此函数用于接收usart6的数据，uart6_buff每次接收一个数据，
//放进USART_STU_BUF.校验完成后放于USART_FRAME中等待处理输出。
//////////将不同模式数据存进对应结构中
void Student_DATA_process() {
	 Frame1 frame1;				//12
	 //Frame2 frame2;
	 //Frame3 frame3;
	 //Frame4 frame4;
	 //Frame5 frame5;
	 //Frame6 frame6;
	 Frame7 frame7;				//24
	 Frame8 frame8;				//23
 	 //Frame9 frame9;
 	 //Frame10 frame10;
	 //Frame11 frame11;
	 Frame12 frame12;			//10
	 Frame13 frame13;			//15
	 //Frame14 frame14;
	 //Frame15 frame15;			//25
	 uint8_t *pFrame;
	 uint8_t i;
	//tick[eJudge] = HAL_GetTick();
	 if(USART_FRAME[0] != 0) {
		 if( USART_LEN == 12){    //decide received data type:1/length 46
				  pFrame = (U8*)&frame1;
					i = 0;
					while(i < USART_LEN) {
						pFrame[i] = USART_FRAME[i];
						i++;
					}
					gameprograss = frame1.extGameState.game_progress;	//当前比赛阶段
					stageremaintime = frame1.extGameState.stage_remain_time;	//当前阶段剩余时间
					USART_FRAME[0] = 0;
			} else if ( USART_LEN == 24){
			    pFrame = (U8*)&frame7;
					i = 0;
			   	while(i < USART_LEN) {
						pFrame[i] = USART_FRAME[i];
						i++;
					}
					robotlevel = frame7.extGameRobotState.robot_level;	//机器人等级
					my_robot_id = frame7.extGameRobotState.robot_id;
					if(robotlevel == 0) robotlevel = 1;
					remainHP = frame7.extGameRobotState.remain_HP;	//机器人剩余血量
					maxHP = frame7.extGameRobotState.max_HP;	//机器人上限血量
					shooterheat0coolingrate = frame7.extGameRobotState.shooter_heat0_cooling_rate;	//17mm枪口每秒冷却值
					shooterheat0coolinglimt = frame7.extGameRobotState.shooter_heat0_cooling_limit;	//17mm枪口热量上限
					shooterheat1coolingrate = frame7.extGameRobotState.shooter_heat1_cooling_rate;	//42mm枪口每秒冷却值
					shooterheat1coolinglimt = frame7.extGameRobotState.shooter_heat1_cooling_limit;	//42mm枪口热量上限
					USART_FRAME[0] = 0;
			}  else if ( USART_LEN == 23){    //decide received data type:1/length 46
				  pFrame = (U8*)&frame8;
					i = 0;
					while(i < USART_LEN) {
						pFrame[i] = USART_FRAME[i];
						i++;
					}
					chassisvolt = frame8.extPowerHeatData.chassis_volt;	//底盘输出电压 mV
					chassiscurrent = frame8.extPowerHeatData.chassis_current;	//底盘输出电流 mA
					chassispower = frame8.extPowerHeatData.chassis_power;	//底盘输出功率 W
					chassispowerbuffer = frame8.extPowerHeatData.chassis_power_buffer;	//底盘功率缓冲 J
					shooterheat0 = frame8.extPowerHeatData.shooter_heat0;	//17mm枪口热量
					shooterheat1 = frame8.extPowerHeatData.shooter_heat1;	//42mm枪口热量
					USART_FRAME[0] = 0;
			} else if( USART_LEN == 10){
					pFrame = (U8*)&frame12;
					i = 0;
					while(i < USART_LEN) {
						pFrame[i] = USART_FRAME[i];
						i++;
					}
					armorid = frame12.extRobotHurt.armor_id;	//收到伤害的装甲片
					hurttype = frame12.extRobotHurt.hurt_type;	//血量变化类型
					USART_FRAME[0] = 0;
		}	else if( USART_LEN == 15){
					pFrame = (U8*)&frame13;
					i = 0;
					while(i < USART_LEN) {
						pFrame[i] = USART_FRAME[i];
						i++;
					}
					bullettype = frame13.extShootData.bullet_type;	//子弹类型
					bulletfreq = frame13.extShootData.bullet_freq;	//子弹射频 Hz
					bulletspeed = frame13.extShootData.bullet_speed;	//子弹射速 m/s
					USART_FRAME[0] = 0;
		}
    }

}
///////////////////////////////////////////////////////////////////
uint8_t judge_uart_rx_buff[50];
void Student_DATA_Receive() {  
	  U8 rd, i;
		rd = judge_uart_rx_buff[0];
	
		if(recLength) {       // 接收到帧头后进入数据接收状态
			USART_STU_BUF[USART_STU_STA++] = rd;
			if(USART_STU_STA == 5) { // 接收完第五位后进行CRC8校验
				recLength = USART_LEN = USART_STU_BUF[2]<<8 | USART_STU_BUF[1] + 9; // 计算数据帧长度，用于设置结束接收（接收完一帧数据）的条件z
				if(!Verify_CRC8_Check_Sum(USART_STU_BUF,USART_STU_STA)) {
					recLength = 0; // 若校验和有误，丢弃并重新接收下一帧
				}
			}
			if(USART_STU_STA == recLength) { // 接收完完整的一帧数据
				if(Verify_CRC16_Check_Sum(USART_STU_BUF,USART_STU_STA)) // CRC16校验
				{
					for(i=0; i<USART_STU_STA; i++) USART_FRAME[i] = USART_STU_BUF[i]; // 复制数据帧到main函数中使用
				  Student_DATA_process();
					judge_tick = HAL_GetTick();
			  }	
					recLength = 0; // 结束数据接收状态
			}
		} else if(rd == 0xA5) { // 非数据接收状态时等待帧头
			USART_STU_STA = 0; // 清空缓存计数器
			for(i=0;i<200;i++) USART_STU_BUF[i] = 0;
			USART_STU_BUF[USART_STU_STA++] = rd; // 将帧头存入缓存
			recLength = 46; // 初步设置接收长度，用于进入接收状
			for(i=0;i<46;i++) USART_FRAME[i] = 0;
		}else{
			recLength = 0;
		}
}	


void Judge_Task_UART_IRQHandler(void) {
	isrflags = READ_REG(judge_huart.Instance->SR);
	cr1its = READ_REG(judge_huart.Instance->CR1);
	int i;
	if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))//RXNE
	{
		judge_uart_rx_buff[0]=(uint8_t)(judge_huart.Instance->DR & (uint8_t)0x00FF);
		//User Code//
		Student_DATA_Receive();
	}
	if(((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET)){
		judge_huart.Instance->DR = (TxBuf1[TxCounter1] & (uint16_t)0x01FFU);
		TxCounter1++;
		if(TxCounter1 == Txcursor1){
			__HAL_UART_DISABLE_IT(&judge_huart,UART_IT_TXE);  
			for(i=0;i<256;i++) TxBuf1[i] = 0;
		}
	}
}


void judge_send_custom_data(float i1,float i2,float i3,uint8_t mask)
{
	Frame15 frame15;
	int i;
	uint8_t * pframe=(uint8_t *)&frame15;
	frame15.SOF=0xA5;
	frame15.data_length=19;
	frame15.seq=1;
	Append_CRC8_Check_Sum((uint8_t*)&frame15,5);
	frame15.cmd_id=0x0301;
	
	
	frame15.extStudentInteractiveHeaderData.data_cmd_id = 0xD180;
	frame15.extStudentInteractiveHeaderData.send_ID = my_robot_id;
	if(my_robot_id<10)
	{
			frame15.extStudentInteractiveHeaderData.receiver_ID = 256+my_robot_id;
	}
	else
	{
		  frame15.extStudentInteractiveHeaderData.receiver_ID = 262+my_robot_id;

	}
	
	frame15.clientCustomData.data1=i1;
	frame15.clientCustomData.data2=i2;
	frame15.clientCustomData.data3=i3;
	frame15.clientCustomData.masks=mask;
	Append_CRC16_Check_Sum((uint8_t*)&frame15,28);

	for(i=0;i<28;i++)
    {
        TxBuf1[Txcursor1++]=*(pframe+i);
    }
    __HAL_UART_ENABLE_IT(&judge_huart,UART_IT_TXE);
}

