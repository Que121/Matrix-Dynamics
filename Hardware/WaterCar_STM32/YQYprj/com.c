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
	SendInfo.head[0] 						= 'S';				//֡ͷ
	SendInfo.head[1] 						= 'T';				//֡ͷ
	SendInfo.head[2] 						= 'M';				//֡ͷ
	SendInfo.GPS_j 							= 'E';				//����
	SendInfo.GPS_w 							= 'N';				//��γ
	SendInfo.GPS_jNum 					= 111.11;			//����  ��������
	SendInfo.GPS_wNum 					= 222.22;			//��γ  ��������
	SendInfo.GPS_h							= 12;					//GPSСʱ
	SendInfo.GPS_m							= 0;					//GPS����
	SendInfo.GPS_s							= 30;					//GPS��
	SendInfo.PowerStatus 				= 0;					//�����Դ�ر�
	SendInfo.PumpStatus 				= 0;					//ˮ�ùر�
	SendInfo.distance 					= 3.1;				//���
	SendInfo.Pitch 							= 45;					//��̨����
	SendInfo.Yaw 								= 30;					//��̨ƫ��
	SendInfo.WaterLevel 				= 100;				//ˮ��ˮλ
	SendInfo.Battery 						= 100;				//��ص���
	SendInfo.ERRcode 						= 2;					//�������
	SendInfo.tail 							= '\n';				//֡β
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
	SendInfo.sendBuffer[21] = SendInfo.timeBuffer >> 8;			//��8λ
	SendInfo.sendBuffer[22] = SendInfo.timeBuffer;
	
	SendInfo.sendBuffer[23] = SendInfo.PowerStatus | (SendInfo.PumpStatus << 1);	//bit0 ��Դ bit1 ˮ��״̬
	
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

void GetInfo_Init(void)				//��ʼ���ṹ��
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

void GetInfo_Dispose(void)	//�������
{
	uint8_t i = 0, p = 0;
	for(; i < 12; i++)
	{
		if((GetInfo.recvBuff[i] == 'E') && (GetInfo.recvBuff[i+1] == 'S') && (GetInfo.recvBuff[i+2] == 'P') && (GetInfo.recvBuff[i + 5] == '\n'))
		{
			GetInfo.Speed_s  = GetInfo.recvBuff[i + 3] & (0x0f);				//��4
			GetInfo.Speed_w  = GetInfo.recvBuff[i + 3] >> 4;						//��4
			GetInfo.Power		 = GetInfo.recvBuff[i + 4] & (1<<0);				//bit0
			GetInfo.Pump		 = (GetInfo.recvBuff[i + 4] & (1<<1)) >> 1;	//bit1
			GetInfo.Light		 = (GetInfo.recvBuff[i + 4] & (1<<2)) >> 2;	//bit2
			GetInfo.recvTime = HAL_GetTick();														//��¼��ȡʱ��
			
			printf("receive success\n");
			printf("speed_s:%d speed_w:%d\n",GetInfo.Speed_s,GetInfo.Speed_w);
			printf("Power:%d Pump:%d Light:%d\n",GetInfo.Power,GetInfo.Pump,GetInfo.Light);
			printf("recvTime:%d\n",GetInfo.recvTime);
			
			for(p = 0; p < 6; p++)GetInfo.recvBuff[i + p] = 0;					//���
		}
	}
}
