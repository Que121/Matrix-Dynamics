/**********************************************************************
项目名称/Project          : 灌溉机器人
团队/Team                : 矩阵动力
作者/Author              : 蔚清洋
日期/Date（YYYYMMDD）     : 20220811
程序目的/Purpose          : 
完成MQTT消息发布
完成与下位机STM32串口通信
-----------------------------------------------------------------------
本程序仅为通信demo
***********************************************************************/
#include <WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>

//测试同步
#include "DHT.h"


#define DHTPIN 12
#define DHTTYPE DHT11   // DHT 11
#define led_Pin  13

// 设置wifi接入信息(请根据您的WiFi信息进行修改)
const char* ssid     = "father";
const char* password = "yqy12345";
//const char* mqttServer = "test.ranye-iot.net";    //暂时使用然也物联
const char* mqttServer = "43.138.132.49";    //暂时使用然也物联
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
typedef struct
{
  uint8_t ID;                                 //小车ID
  char GPS_J,GPS_W;                           //NS EW
  double GPS_Jnum,GPS_Wnum;                   //具体纬度
  uint8_t GPSTime[3];                         //时分秒
  float DhtHumidity,DhtTemperature;           //温湿度
  uint8_t Power,Pump,Water,Bat;               //车辆信息
  float Dis,Pitch,Yaw;                        //车辆信息
  uint8_t errCode;
}PubMessage_t;
typedef struct
{
  char head[3];
  char head_MQTT[3];
  uint8_t ID;
  uint8_t speed_s, speed_w;        //直行\转向
  uint8_t Power, Pump, Light;
  char tail;
}toSTM32info_t;
toSTM32info_t toSTM32info;
uint8_t STM_recvBuff[40] = {0};
uint8_t STM_recvBuff_Pin = 0, STM_recvBuff_ShouldPlus = 0;

PubMessage_t PubMessage;

DHT dht(DHTPIN, DHTTYPE);
Ticker ticker;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
 
int count = 0;    // Ticker计数用变量
int count_SendTostm32 = 0;

void PubMessage_Init(PubMessage_t* PubM);        //初始化结构体信息
String getPubMessage(PubMessage_t* PubM);        //获得MQTT推送字符
void toSTM32info_Init(void);
void toSTM32info_SendMessage(void);
void toSTM32info_ResetMessage(void);          //清空之前的数据
void fromSTM32info_GetMessage(void);      //从串口缓冲区查找
void fromSTM32info_DecodeMessage(void);   //从stm32发来的信息里解码
void setup() {
  Serial.begin(115200);
  delay(1);
  toSTM32info_Init();
  pinMode(led_Pin, OUTPUT);
  dht.begin();
 
  // Ticker定时对象
  ticker.attach(0.1, tickerCount);
  //设置ESP8266工作模式为无线终端模式
  WiFi.mode(WIFI_STA);
  
  // 连接WiFi
  connectWifi();
  
  // 设置MQTT服务器和端口号
  mqttClient.setServer(mqttServer, 1883);
  mqttClient.setCallback(receiveCallback);
 
  // 连接MQTT服务器
  connectMQTTServer();
  PubMessage_Init(&PubMessage);
}
 
void loop() {
  if (mqttClient.connected()) { // 如果开发板成功连接服务器
    // 每隔3秒钟发布一次信息
    if (count >= 30){
      PubMessage.DhtTemperature = dht.readTemperature();
      PubMessage.DhtHumidity = dht.readHumidity();
      pubMQTTmsg();
      count = 0;
    }    
    // 保持心跳
    mqttClient.loop();
  } else {                  // 如果开发板未能成功连接服务器
    connectMQTTServer();    // 则尝试连接服务器
  }
  if(count_SendTostm32 >= 2)    //200ms和stm32通信一次
  {
    toSTM32info_SendMessage();
    toSTM32info_ResetMessage();
    count_SendTostm32 = 0;
  }
  
  fromSTM32info_GetMessage();
}
 
void tickerCount(){
  count++;
  count_SendTostm32++;
}
void toSTM32info_ResetMessage(void){
  toSTM32info.speed_s = 0;
  toSTM32info.speed_w = 0;
  toSTM32info.Power   = 0;
  toSTM32info.Pump    = 0;
}
void connectMQTTServer(){
  // 根据ESP8266的MAC地址生成客户端ID（避免与其它ESP8266的客户端ID重名）
  String clientId = "BJFU_esp32-" + WiFi.macAddress();
 
  // 连接MQTT服务器
  if (mqttClient.connect(clientId.c_str())) { 
    Serial.println("MQTT Server Connected.");
    Serial.println("Server Address: ");
    Serial.println(mqttServer);
    Serial.println("ClientId:");
    Serial.println(clientId);
    subscribeTopic(); // 订阅指定主题
  } else {
    Serial.print("MQTT Server Connect Failed. Client State:");
    Serial.println(mqttClient.state());
    delay(3000);
  }   
}
 
// 发布信息
void pubMQTTmsg(){
  String topicString = "car_state";
  char publishTopic[topicString.length() + 1];  
  strcpy(publishTopic, topicString.c_str());
 
  // 定时向服务器主题发布当前D3引脚状态
  String messageString = getPubMessage(&PubMessage);

  
  char publishMsg[messageString.length() + 1];   
  strcpy(publishMsg, messageString.c_str());
  
  // 实现ESP8266向主题发布信息
  if(mqttClient.publish(publishTopic, publishMsg)){
    Serial.print("Publish Topic:");Serial.println(publishTopic);
    Serial.print("Publish message:");Serial.println(publishMsg);    
  } else {
    Serial.println("Message Publish Failed."); 
  }
}
 
// 订阅指定主题
void subscribeTopic(){
 
  // 建立订阅主题。主题名称以Taichi-Maker-Sub为前缀，后面添加设备的MAC地址。
  // 这么做是为确保不同设备使用同一个MQTT服务器测试消息订阅时，所订阅的主题名称不同
  String topicString = "car_control2";
  char subTopic[topicString.length() + 1];  
  strcpy(subTopic, topicString.c_str());
  
  // 通过串口监视器输出是否成功订阅主题以及订阅的主题名称
  if(mqttClient.subscribe(subTopic)){
    Serial.println("Subscribe Topic:");
    Serial.println(subTopic);
  } else {
    Serial.print("Subscribe Fail...");
  }  
}
 
// 收到信息后的回调函数
void receiveCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message Received [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) 
    Serial.print((char)payload[i]);
  Serial.println("");
  Serial.print("Message Length(Bytes) ");
  Serial.println(length);

//MQTT信息解码
  for (int i = 0; i < 3; i++) 
    toSTM32info.head_MQTT[i] = (char)payload[i];

  if((toSTM32info.head_MQTT[0] == 'C') && (toSTM32info.head_MQTT[1] == 'T') && (toSTM32info.head_MQTT[2] == 'R'))
  {
    toSTM32info.ID = ((char)payload[3] - '0')*10 + ((char)payload[4] - '0');
    toSTM32info.speed_s =  (char)payload[5] - '0';   //前进速度
    toSTM32info.speed_w =  (char)payload[6] - '0';   //转向速度
    toSTM32info.Power   =  (char)payload[7];   //电源
    toSTM32info.Pump    =  (char)payload[8];   //水泵
    Serial.println("decode MQTT success:");
    Serial.print("speed(s/w)");
    Serial.println(toSTM32info.speed_s);
    Serial.println(toSTM32info.speed_s);
    if((toSTM32info.Power == '1') || (toSTM32info.Power == 1)){
      toSTM32info.Power = 1;
      Serial.print("power on!  ");
    }else{
      Serial.print("power off  ");
    }
    if((toSTM32info.Pump == '1') || (toSTM32info.Pump == 1)){
      toSTM32info.Power = 1;
      Serial.println("Pump on!");
    }else{
      Serial.println("Pump off");
    }
  }else{
    Serial.println("decode MQTT failed");
  }

 
  // if ((char)payload[0] == '1') {     // 如果收到的信息以“1”为开始
  //   digitalWrite(led_Pin, HIGH);  // 则点亮LED。
  //   Serial.println("led on!");
  // } else {                           
  //   digitalWrite(led_Pin, LOW); // 否则熄灭LED。
  //   Serial.println("led off!");
  // }
}
 
// ESP32连接wifi
void connectWifi(){
 
  WiFi.begin(ssid, password);
 
  //等待WiFi连接,成功连接后输出成功信息
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Connected!");  
  Serial.println(""); 
}

void PubMessage_Init(PubMessage_t* PubM)
{
  uint8_t i = 0;
    PubM->ID              = 1;            //小车ID1
    PubM->GPS_J           = 'E';          //东 经
    PubM->GPS_W           = 'N';          //北 纬
    PubM->GPS_Jnum        = 11919.42;     //经度
    PubM->GPS_Wnum        = 2605.30;      //纬度
    PubM->DhtHumidity     = 0;            //湿度
    PubM->DhtTemperature  = 0;            //温度
  for(;i < 3; i++)PubM->GPSTime[i] = 0;   //GPS授时
    PubM->Power           = 0;            //车辆信息 电源
    PubM->Pump            = 0;            //车辆信息 水泵
    PubM->Water           = 0;            //车辆信息 剩余水量
    PubM->Dis             = 0;            //车辆信息 测距
    PubM->Pitch           = 0;            //车辆信息 云台Pitch
    PubM->Yaw             = 0;            //车辆信息 云台Yaw
    PubM->Bat             = 0;            //车辆信息 电池电量
    PubM->errCode         = 0;            //错误码，0无错误
}

String getPubMessage(PubMessage_t* PubM)
{
  String Temp;
  char GPS_c[37];
  char num_u8[3];
  char num_u82[2];
  char num_f32[6];
  char num_f21[4];
  sprintf(num_u82,"%02d",PubM->ID);
  Temp += "ID:" + String(num_u82) + "_";     //添加头
  //GPS相关信息
  sprintf(GPS_c,"%c%07.2lf%c%07.2lfGTime:%02d:%02d:%02d",PubM->GPS_J,PubM->GPS_Jnum,PubM->GPS_W,PubM->GPS_Wnum\
      ,PubM-> GPSTime[0], PubM-> GPSTime[1], PubM-> GPSTime[2]);
  Temp += "GPS:" + String(GPS_c) + "_";
  //车辆状态信息
  if(PubM->Power == 1)Temp += "ON_";else Temp += "OFF_";
  if(PubM->Pump  == 1)Temp += "ON_"; else Temp += "OFF_";
  sprintf(num_u8,"%03d",PubM->Water);
  Temp += "Wt:" + String(num_u8) + "_";
  sprintf(num_u8,"%03.2f",PubM->Dis);
  Temp += "D:" + String(num_u8) + "_";
  sprintf(num_u82,"%02.1f",PubM->Pitch);
  Temp += "P:" + String(num_u82);
  sprintf(num_u82,"%02.1f",PubM->Yaw);
  Temp += "Y:" + String(num_u82);
  sprintf(num_u8,"%03d",PubM->Bat);
  Temp += "B:" + String(num_u8) + "_";
  //温湿度
  sprintf(num_f21,"%02.1f",PubM->DhtTemperature);
  Temp += "T:" + String(num_f21) + "_";
  sprintf(num_f21,"%02.1f",PubM->DhtHumidity);
  Temp += "H:" + String(num_f21) + "_";
  sprintf(num_u82,"%02d",PubM->errCode);
  if(PubM->errCode == 0)Temp += "Ready";
  else Temp += "Error" + String(num_u82);
  return Temp;
}
void fromSTM32info_DecodeMessage(void)   //从stm32发来的信息里解码
{
  uint16_t u16_temp = 0;
  PubMessage.GPS_J = STM_recvBuff[3];
  PubMessage.GPS_W = STM_recvBuff[4];

  for(int i = 0;i < 8; i ++)LFtoU8.u8Num[i] = STM_recvBuff[5 + i];
  PubMessage.GPS_Jnum = LFtoU8.lfNum;
  for(int i = 0;i < 8; i ++)LFtoU8.u8Num[i] = STM_recvBuff[13 + i];
  PubMessage.GPS_Wnum = LFtoU8.lfNum;
  u16_temp = ((STM_recvBuff[21]<<8)|(STM_recvBuff[22])) ;

  PubMessage.GPSTime[1] = u16_temp/100;     //分钟
  PubMessage.GPSTime[2] = (u16_temp%100)/1;       //秒
  PubMessage.Power = (STM_recvBuff[23] & (1 << 0));//bit0 电源 bit1 水泵状态
  PubMessage.Pump  = (STM_recvBuff[23] & (1 << 1)) >> 1;
  
  for(int i = 0;i < 4; i ++)FtoU8.u8Num[i] = STM_recvBuff[24 + i];
  PubMessage.Dis = FtoU8.fNum;
  for(int i = 0;i < 4; i ++)FtoU8.u8Num[i] = STM_recvBuff[28 + i];
  PubMessage.Pitch = FtoU8.fNum;
  for(int i = 0;i < 4; i ++)FtoU8.u8Num[i] = STM_recvBuff[32 + i];
  PubMessage.Yaw = FtoU8.fNum;

  PubMessage.Water    = STM_recvBuff[36];
  PubMessage.Bat      = STM_recvBuff[37];
  PubMessage.errCode  = STM_recvBuff[38];

  //--------------------------------------打出来看看？------------------------------------------
  char pBuff[20];
  Serial.println("\n------------------RAW Data:----------------");
  for(int i = 0; i < 40; i++)
  {
    sprintf(pBuff,"%x ",STM_recvBuff[i]);Serial.write(pBuff);
  }
  Serial.println("\n----------Decode Message Success:----------");
  sprintf(pBuff,"HEAD:%c%c%c\n",STM_recvBuff[0],STM_recvBuff[1],STM_recvBuff[2]);Serial.write(pBuff);
  Serial.print("GPS:");
//  sprintf(pBuff,"%d\n",(int)PubMessage.GPS_Jnum);Serial.write(pBuff);
  sprintf(pBuff,"%c%07.2f ",PubMessage.GPS_J,PubMessage.GPS_Jnum);Serial.write(pBuff);
  sprintf(pBuff," %c%7.2f\n",PubMessage.GPS_W,PubMessage.GPS_Wnum);Serial.print(pBuff);
  sprintf(pBuff,"GT:m%02d s:%02d\n",PubMessage.GPSTime[1],PubMessage.GPSTime[2]);Serial.write(pBuff);
  
  sprintf(pBuff,"Power:%d\n",PubMessage.Power);Serial.write(pBuff,8);
  sprintf(pBuff,"Pump :%d\n",PubMessage.Pump);Serial.write(pBuff,8);
  sprintf(pBuff,"Water:%d\n",PubMessage.Water);Serial.write(pBuff,8);
  sprintf(pBuff,"Bat :%d\n",PubMessage.Bat);Serial.write(pBuff,8);
  sprintf(pBuff,"errCode :%d\n",PubMessage.errCode);Serial.write(pBuff,8);
  
  Serial.print("Dis:");Serial.println(PubMessage.Dis);
  Serial.print("Pitch:");Serial.println(PubMessage.Pitch);
  Serial.print("Yaw:");Serial.println(PubMessage.Yaw);
  Serial.print("errCode:");Serial.println(PubMessage.errCode);
  Serial.print("millis:");Serial.println(millis());
  Serial.println("\n------------Decode finish-------------------");


  for(int i = 0; i < 40; i++)STM_recvBuff[i] = 0;       //冲厕所
  
}
void fromSTM32info_GetMessage(void)    //从串口缓冲区查找
{
  if(Serial2.available())
  {
    STM_recvBuff[STM_recvBuff_Pin] = Serial2.read(); //先读取第一个字节判断
    if( STM_recvBuff_ShouldPlus >= 3) STM_recvBuff_Pin ++;
    if((STM_recvBuff_ShouldPlus == 0) && (STM_recvBuff[0] == 'S')){STM_recvBuff_Pin ++; STM_recvBuff_ShouldPlus = 1;Serial.println("H1 ");}
    if((STM_recvBuff_ShouldPlus == 1) && (STM_recvBuff[1] == 'T')){STM_recvBuff_Pin ++; STM_recvBuff_ShouldPlus = 2;Serial.println("H2 ");}
    if((STM_recvBuff_ShouldPlus == 2) && (STM_recvBuff[2] == 'M')){STM_recvBuff_Pin ++; STM_recvBuff_ShouldPlus = 3;Serial.println("H3 ");}
    if(STM_recvBuff_Pin == 40){STM_recvBuff_Pin = 0; STM_recvBuff_ShouldPlus = 0; fromSTM32info_DecodeMessage();}   //完成整帧获取，进行解码

    if(STM_recvBuff_ShouldPlus == 3){Serial.println("Get STM Message Head!"); STM_recvBuff_ShouldPlus = 4;}
  }
}
void toSTM32info_Init(void)
{
  Serial2.begin(115200);
  //Serial2.println("hello YQY test serial2");
  toSTM32info.head[0] = 'E';
  toSTM32info.head[1] = 'S';
  toSTM32info.head[2] = 'P';
  toSTM32info.speed_s = 8;
  toSTM32info.speed_w = 2;
  toSTM32info.Power = 1;
  toSTM32info.Pump = 0;
  toSTM32info.Light = 1;
  toSTM32info.tail = '\n';
}
void toSTM32info_SendMessage(void)
{
  char sendBuff[6];
  for(int i = 0; i < 3; i++)sendBuff[i] = toSTM32info.head[i];
  sendBuff[3] = (toSTM32info.speed_s | (toSTM32info.speed_w << 4));
  sendBuff[4] = (toSTM32info.Power | (toSTM32info.Pump << 1) | (toSTM32info.Light << 2));
  sendBuff[5] = toSTM32info.tail;
  Serial2.write(sendBuff,6);
}
