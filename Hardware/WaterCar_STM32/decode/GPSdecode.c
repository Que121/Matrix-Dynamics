#include <stdio.h>
#include "GPSdecode.h"
#include "main.h"

#define PRINT_EN		0
Data_t GPS_data;
char temp_buff[100] = "$GPGGA,012916.00,4046.46475,N,11141.38758,E,1,07,1.19,1099.1,M,-23.8,M,,*4C";
char IsdecodeInit = 0;

void decodeGPS(char* buffer, int num)
{
	int pinHead = 0, pinTail = 0, i;
	if(IsdecodeInit == 0)							//init all buffer and information
	{
		IsdecodeInit 				= 1;				//close the door
		GPS_data.jingdu 		= 'E';
		GPS_data.weidu 			= 'N';
		GPS_data.jingdu_num =	123.456;
		GPS_data.weidu_num 	=	789.000;
		GPS_data.GTime[0] 	= 11;
		GPS_data.GTime[1] 	= 10;
		GPS_data.GTime[2] 	= 13;
		GPS_data.IsGPS_OK 	= GPS_ERR;
		return;
	}

	for(i = 0; i < num; i++)					//search head "$GPGGA"
	{
		if(buffer[i] == '$' && buffer[i+3] == 'G' &&buffer[i+4] == 'G' &&buffer[i+5] == 'A')
		{
			pinHead = i + 7;
			break;
		}
	}
	
	GPS_data.GTime[0] = (buffer[pinHead + 0] - '0') * 10 + (buffer[pinHead + 1] - '0');
	GPS_data.GTime[1] = (buffer[pinHead + 2] - '0') * 10 + (buffer[pinHead + 3] - '0');
	GPS_data.GTime[2] = (buffer[pinHead + 4] - '0') * 10 + (buffer[pinHead + 5] - '0');
	if(PRINT_EN)printf("GTime %02d:%02d:%02d\n", GPS_data.GTime[0],GPS_data.GTime[1],GPS_data.GTime[2]);

	pinHead += 10;
	for(i = pinHead; i < num; i++)
	{
		if(buffer[i] == ',')
		{
			pinTail = i - 1;
			break;
		}
	}
//	printf("head:%d tail:%d length:%d\n",pinHead, pinTail, pinTail - pinHead + 1);
	GPS_data.jingdu_num = decodeChar(buffer + pinHead, pinTail - pinHead + 1);			
	GPS_data.jingdu = buffer[pinTail + 2];
	if(PRINT_EN)printf("%c %lf\n", GPS_data.jingdu,GPS_data.jingdu_num);

	pinHead = pinTail + 4;
	for(i = pinHead; i < num; i++)
	{
		if(buffer[i] == ',')
		{
			pinTail = i - 1;
			break;
		}
	}
//	printf("head:%d c%c tail:%d c%c length:%d\n",pinHead,buffer[pinHead] , pinTail, buffer[pinTail], pinTail - pinHead + 1);
	GPS_data.weidu_num = decodeChar(buffer + pinHead, pinTail - pinHead + 1);		
	GPS_data.weidu = buffer[pinTail + 2];
	if(PRINT_EN)printf("%c %lf\n", GPS_data.weidu,GPS_data.weidu_num);
	if((GPS_data.GTime[0] <= 59) && (GPS_data.GTime[1] <= 59) && (GPS_data.GTime[2] <= 59) && (buffer[pinTail + 4] == '1')) 
	{
		GPS_data.IsGPS_OK = GPS_OK;
	}
	if(PRINT_EN)printf("GPSstatus %d",GPS_data.IsGPS_OK);
	
	
}
double decodeChar(char* buffer, int length)
{
	int pinHead = 0, pinTail = 0, i = 0;
	double tempNum;											//to calculate decimals
	double returnNum = 0;
//	//show raw data
//	printf("raw data:");
//	for(i = 0; i < length; i++)printf("%c", buffer[i]);
//	printf("\n");

	for(i = pinHead; i < length; i++)						//find tail before '.'
	{
		if(buffer[i] == '.')
		{
			pinTail = i - 1;
			break;
		}
	}
	for(i = 0;i <= pinTail; i++)							//calculate integer
	{
		returnNum += (buffer[i] - '0');
		if(i != pinTail)returnNum *= 10;					//the last one do not need
	}
	pinHead = pinTail + 2;									//jump out . and point to new num

	for(i = length-1;i >= pinHead; i--)
	{
		tempNum += (buffer[i] - '0');
		tempNum *= 0.1f;									//the last one do not need
	}
	returnNum += tempNum;									//add decimals
//	printf("return :%lf\n", returnNum);
	return returnNum;
}

void decodeGPS_showResult(void)
{
	GPS_data.IsGPS_OK = 0;
	printf("\nGPSresult: ");
	printf("GTime %02d:%02d:%02d JW:%c %lf %c %lf", GPS_data.GTime[0], GPS_data.GTime[1], GPS_data.GTime[2], GPS_data.jingdu, \
		GPS_data.jingdu_num,GPS_data.weidu,GPS_data.weidu_num);
	
}
#define FAKE_GPS_JING (57.14159f)
#define FAKE_GPS_WEI  (69.27834f)
void decodeGPS_fakeHandle(void)
{
	static uint32_t lastTime = 0;
	static int8_t 	i = -10;
	if(HAL_GetTick() - lastTime > 1000)
	{
		lastTime = HAL_GetTick();
		GPS_data.GTime[2] ++;
		if(GPS_data.GTime[2] > 59)
		{
			GPS_data.GTime[2] = 0;
			GPS_data.GTime[1] ++;
		}
		if(GPS_data.GTime[1] > 59)
		{
			GPS_data.GTime[1] = 0;
			GPS_data.GTime[0] ++;
		}
		if(GPS_data.GTime[0] > 59)
		{
			GPS_data.GTime[0] = 0;
		}
		
		GPS_data.jingdu_num = FAKE_GPS_JING + 0.0007 * i * i;
		GPS_data.weidu_num  = FAKE_GPS_WEI  + 0.0012 * i * i;
		
		if(++i > 50)i = -50;
		
	}
}

/*
int main()
{
//	printf("test decodeChar%lf\n",decodeChar("11141.38758", 11));
//	printf("test decodeChar%lf\n",decodeChar("12345.38758", 11));
//	printf("test decodeChar%lf\n",decodeChar("67890.38758", 11));
	decodeGPS(temp_buff, 70);
	return 0;
}
*/

