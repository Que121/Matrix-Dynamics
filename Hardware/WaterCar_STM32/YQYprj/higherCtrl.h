#ifndef __higherCtrl_h
#define __higherCtrl_h
#include "main.h"





extern void higherCtrl_Init(void);
extern void higherCtrl_Handle(void);


extern void MotorCtrl(int8_t Left, int8_t Right);			//from -10 to 10
extern void PumpCtrl(uint8_t onOff);									//on 1    off 0
extern void GimbalCtrl(uint8_t Pitch, uint8_t Yaw);		//from 0 to 180
extern void updateInformation_Handle(void);						//更新状态数据到esp32
#endif

