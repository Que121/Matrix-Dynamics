#include "com.h"
#include "stdio.h"
#include "usart.h"

SendInfo_t SendInfo;
GetInfo_t	GetInfo;

union floatToUint8_t4
{
    float fNum;
    uint8_t u8Num[4];
}FtoU8;
union doubleToUint8_t8
{
    double lfNum;
    uint8_t u8Num[8];
}LFtoU8;

void SendInfo_Init(void)
{
	SendInfo.head[0] 						= 'S';				//帧头
	SendInfo.head[1] 						= 'T';				//帧头
	SendInfo.head[2] 						= 'M';				//帧头
	SendInfo.GPS_j 							= 'E';				//东经
	SendInfo.GPS_w 							= 'N';				//北纬
	SendInfo.GPS_jNum 					= 111.11;			//东经  测试数据
	SendInfo.GPS_wNum 					= 222.22;			//北纬  测试数据
	SendInfo.GPS_h							= 12;					//GPS小时
	SendInfo.GPS_m							= 0;					//GPS分钟
	SendInfo.GPS_s							= 30;					//GPS秒
	SendInfo.PowerStatus 				= 0;					//电机电源关闭
	SendInfo.PumpStatus 				= 0;					//水泵关闭
	SendInfo.distance 					= 3.1;				//测距
	SendInfo.Pitch 							= 45;					//云台俯仰
	SendInfo.Yaw 								= 30;					//云台偏航
	SendInfo.WaterLevel 				= 100;				//水箱水位
	SendInfo.Battery 						= 100;				//电池电量
	SendInfo.ERRcode 						= 2;					//错误代码
	SendInfo.tail 							= '\n';				//帧尾
}

void SendInfo_Send(void)
{
	uint8_t i = 0;
	for(i = 0;i < 3; i ++)SendInfo.sendBuffer[i] = SendInfo.head[i];
	SendInfo.sendBuffer[3] = SendInfo.GPS_j;
	SendInfo.sendBuffer[4] = SendInfo.GPS_w;
	
	LFtoU8.lfNum = SendInfo.GPS_jNum;
	for(i = 0;i < 8; i ++)SendInfo.sendBuffer[5 + i] = LFtoU8.u8Num[i];
	LFtoU8.lfNum = SendInfo.GPS_wNum;
	for(i = 0;i < 8; i ++)SendInfo.sendBuffer[13 + i] = LFtoU8.u8Num[i];
	
	SendInfo.timeBuffer =  SendInfo.GPS_m * 100 + SendInfo.GPS_s * 1;
	SendInfo.sendBuffer[21] = SendInfo.timeBuffer >> 8;			//高8位
	SendInfo.sendBuffer[22] = SendInfo.timeBuffer;
	
	SendInfo.sendBuffer[23] = SendInfo.PowerStatus | (SendInfo.PumpStatus << 1);	//bit0 电源 bit1 水泵状态
	
	FtoU8.fNum = SendInfo.distance;
	for(i = 0;i < 4; i ++)SendInfo.sendBuffer[24 + i] = FtoU8.u8Num[i];
	FtoU8.fNum = SendInfo.Pitch;
	for(i = 0;i < 4; i ++)SendInfo.sendBuffer[28 + i] = FtoU8.u8Num[i];
	FtoU8.fNum = SendInfo.Yaw;
	for(i = 0;i < 4; i ++)SendInfo.sendBuffer[32 + i] = FtoU8.u8Num[i];
	
	SendInfo.sendBuffer[36] = SendInfo.WaterLevel;
	SendInfo.sendBuffer[37] = SendInfo.Battery;
	SendInfo.sendBuffer[38] = SendInfo.ERRcode;
	SendInfo.sendBuffer[39] = SendInfo.tail;
	
	HAL_UART_Transmit(&huart3,SendInfo.sendBuffer,40, 0xFF);		//B11 -> USART3		RX  B10 -> USART3		TX
	//HAL_UART_Transmit_DMA(&huart3,SendInfo.sendBuffer,40);
}

void GetInfo_Init(void)				//初始化结构体
{
	uint8_t i = 0;
	GetInfo.Speed_s = 0;
	GetInfo.Speed_w = 0;
	GetInfo.Pump = 0;
	GetInfo.Power = 0;
	GetInfo.Light = 0;
	GetInfo.recvTime = 0;
	
	for(i = 0; i < 12; i++)GetInfo.recvBuff[i] = 0;
	HAL_UART_Receive_DMA(&huart3, GetInfo.recvBuff, 13);
}

void GetInfo_Dispose(void)	//处理接收
{
	uint8_t i = 0, p = 0;
	for(; i < 12; i++)
	{
		if((GetInfo.recvBuff[i] == 'E') && (GetInfo.recvBuff[i+1] == 'S') && (GetInfo.recvBuff[i+2] == 'P') && (GetInfo.recvBuff[i + 5] == '\n'))
		{
			GetInfo.Speed_s  = GetInfo.recvBuff[i + 3] & (0x0f);				//低4
			GetInfo.Speed_w  = GetInfo.recvBuff[i + 3] >> 4;						//高4
			GetInfo.Power		 = GetInfo.recvBuff[i + 4] & (1<<0);				//bit0
			GetInfo.Pump		 = (GetInfo.recvBuff[i + 4] & (1<<1)) >> 1;	//bit1
			GetInfo.Light		 = (GetInfo.recvBuff[i + 4] & (1<<2)) >> 2;	//bit2
			GetInfo.recvTime = HAL_GetTick();														//记录获取时间
			
			printf("receive success\n");
			printf("speed_s:%d speed_w:%d\n",GetInfo.Speed_s,GetInfo.Speed_w);
			printf("Power:%d Pump:%d Light:%d\n",GetInfo.Power,GetInfo.Pump,GetInfo.Light);
			printf("recvTime:%d\n",GetInfo.recvTime);
			
			for(p = 0; p < 6; p++)GetInfo.recvBuff[i + p] = 0;					//清空
		}
	}
}
