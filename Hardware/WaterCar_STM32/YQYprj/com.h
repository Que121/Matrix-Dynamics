#ifndef __COM_H
#define __COM_H
#include "main.h"

typedef struct
{
	char head[3];
	char GPS_j, GPS_w;								//经度 纬度
	double GPS_jNum, GPS_wNum;				//经度 纬度 数值
	uint8_t GPS_h, GPS_m, GPS_s;			//GPS授时，小时分钟秒
	uint8_t PowerStatus;							//小车是否上电
	uint8_t PumpStatus;								//水泵状态
	float distance;										//测距距离
	float Pitch, Yaw;									//云台
	uint8_t WaterLevel;								//水位 0-100
	uint8_t Battery;									//电量 0-100
	uint8_t ERRcode;									//故障代码
	char tail;												//帧尾
	
	uint16_t timeBuffer;							//时间缓冲区
	uint8_t sendBuffer[40];						//发送缓冲区
	
}SendInfo_t;

typedef struct
{
	uint8_t Speed_s;									//前进速度
	uint8_t Speed_w;									//转向速度
	uint8_t Power;										//电源控制
	uint8_t	Pump;											//水泵控制
	uint8_t	Light;										//照明控制
	
	uint32_t recvTime;								//接收时间
	uint8_t recvBuff[13];							//缓冲区
}GetInfo_t;

extern SendInfo_t SendInfo;
extern GetInfo_t	GetInfo;

extern void SendInfo_Init(void);		//初始化结构体
extern void SendInfo_Send(void);		//发送报文到ESP32

extern void GetInfo_Init(void);			//初始化结构体 初始化接收中断
extern void GetInfo_Dispose(void);	//处理接收

#endif	//__COM_H
