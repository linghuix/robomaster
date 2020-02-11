#include "usart.h"
#include "HW_dbus.h"
#include "string.h"
#include "control_task.h"

#define U8	unsigned char
#define U16	unsigned short int
#define U32	unsigned int
#define judge_huart huart2

//Part: Student_interface
extern uint8_t judge_uart_rx_buff[50];
extern uint8_t USART_STU_BUF[200];     //学生接口接收缓冲,最大200个字节.
extern uint16_t USART_STU_STA;       //学生接口接收状态标记		
extern uint8_t USART_FRAME[46];
extern uint8_t USART_LEN;
extern uint8_t recLength;

extern uint8_t gameprograss;					//当前比赛阶段
extern uint16_t stageremaintime;			//当前阶段剩余时间
extern uint8_t my_robot_id;
extern uint8_t robotlevel;						//机器人等级
extern uint16_t remainHP;						//机器人剩余血量
extern uint16_t maxHP;								//机器人上限血量
extern uint16_t shooterheat0coolingrate;		//17mm枪口每秒冷却值
extern uint16_t shooterheat0coolinglimt;		//17mm枪口每秒热量上限
extern uint16_t shooterheat1coolingrate;		//42mm枪口每秒冷却值
extern uint16_t shooterheat1coolinglimt;		//42mm枪口每秒热量上限
extern uint16_t chassisvolt;					//底盘输出电压 mV
extern uint16_t chassiscurrent;			//底盘输出电流 mA
extern float chassispower;						//底盘输出功率 W
extern uint16_t chassispowerbuffer;	//底盘功率缓冲 J
extern uint16_t shooterheat0;				//17mm枪口热量
extern uint16_t shooterheat1;				//42mm枪口热量
extern uint8_t armorid;							//收到伤害的装甲片
extern uint8_t hurttype;							//血量变化类型
extern uint8_t	bullettype;						//子弹类型
extern uint8_t	bulletfreq;						//子弹射频 Hz
extern float	bulletspeed;						//子弹射速 m/s

//void HAL_UART6_IRQHandler(void);
void Judge_Task_UART_IRQHandler(void);
void judge_send_custom_data(float i1,float i2,float i3,uint8_t mask);

