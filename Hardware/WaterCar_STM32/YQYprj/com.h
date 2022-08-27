#ifndef __COM_H
#define __COM_H
#include "main.h"

typedef struct
{
	char head[3];
	char GPS_j, GPS_w;								//���� γ��
	double GPS_jNum, GPS_wNum;				//���� γ�� ��ֵ
	uint8_t GPS_h, GPS_m, GPS_s;			//GPS��ʱ��Сʱ������
	uint8_t PowerStatus;							//С���Ƿ��ϵ�
	uint8_t PumpStatus;								//ˮ��״̬
	float distance;										//������
	float Pitch, Yaw;									//��̨
	uint8_t WaterLevel;								//ˮλ 0-100
	uint8_t Battery;									//���� 0-100
	uint8_t ERRcode;									//���ϴ���
	char tail;												//֡β
	
	uint16_t timeBuffer;							//ʱ�仺����
	uint8_t sendBuffer[40];						//���ͻ�����
	
}SendInfo_t;

typedef struct
{
	uint8_t Speed_s;									//ǰ���ٶ�
	uint8_t Speed_w;									//ת���ٶ�
	uint8_t Power;										//��Դ����
	uint8_t	Pump;											//ˮ�ÿ���
	uint8_t	Light;										//��������
	
	uint32_t recvTime;								//����ʱ��
	uint8_t recvBuff[13];							//������
}GetInfo_t;

extern SendInfo_t SendInfo;
extern GetInfo_t	GetInfo;

extern void SendInfo_Init(void);		//��ʼ���ṹ��
extern void SendInfo_Send(void);		//���ͱ��ĵ�ESP32

extern void GetInfo_Init(void);			//��ʼ���ṹ�� ��ʼ�������ж�
extern void GetInfo_Dispose(void);	//�������

#endif	//__COM_H
