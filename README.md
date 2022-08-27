# Matrix-Dynamics

此项目为北京林业大学-东方瑞通实训项目，以下为项目详情：

## 一.项目平台 :fa-star: 

### 1.基本要求
1. ubuntu20.04 版本云服务器 （本项目使用服务器 x 3）
2. MySQL8.0 版本云数据库
3. stm32
4. esp32 cam s2
5. openmv4 plus
### 2.开发软件

1. vscode
2. pycharm 2022专业版
3. solidworks 2020
4. keil5MDK



## 二、项目目录 :fa-th-list: 


```
.
├── Back-end              后端代码（前端已集成）                       
│   ├── app.py
│   ├── blueprints
│   ├── config.py
│   ├── decorators.py
│   ├── exts.py
│   ├── migrations
│   ├── models.py
│   ├── MQTT
│   ├── Mqtt_demo
│   ├── __pycache__
│   ├── requirements.txt
│   ├── static
│   ├── templates
│   ├── test.txt
│   └── venv
├── Front-end              前端代码
│   ├── about
│   ├── devicebounded1
│   ├── devicebounded2-4
│   ├── exit
│   ├── get_back_password
│   ├── index
│   ├── purchase
│   ├── register_1
│   └── team
├── Hardware               硬件代码
│   ├── MQTTWcarV1_demo    esp32cam代码
│   ├── openmv             openmv4plus部分代码
│   └── WaterCar_STM32     stm32代码
├── LICENSE
├── Mqtt_demo              mqtt通讯demo
│   ├── pub.py
│   ├── __pycache__
│   ├── sub.py
│   └── test.py
├── README.md
├── Solidworks-drawings    小车solidworks建模文件
│   ├── 9G舵机.SLDPRT
│   ├── 成品1
│   ├── 履带底盘.SLDPRT
│   ├── 水泵.SLDPRT
│   ├── 液位传感器.SLDPRT
│   ├── ESP32-CAM.SLDPRT
│   └── GPS天线.SLDPRT

```
## 三、服务器部署
本项目皆使用腾讯云服务器
### 1.mqtt服务器
首先要在服务器开启1883和18083端口

打开终端执行以下命令
```
wget https://www.emqx.io/downloads/broker/v4.0.5/emqx-ubuntu16.04-v4.0.5.zip
```
解压EMQ X Broker
```
unzip emqx-ubuntu16.04-v4.0.5.zip
```
进入解压出来的emqx文件夹，使用命令启动即可
```
sudo ./bin/emqx start
```
在浏览器输入服务器公网ip加18083即可进入后端管理界面
```
ip:18083
```

默认账号为admin

密码为public
### 2.obs服务器
首先要在服务器开启1935和8090端口

### 3.database服务器
### 4.web服务器（主服务器）

## 四、硬件部署
### 1.物料统计

小车整体采用成品改装+3D打印的方式进行，电路采用飞线，速度快。

后期制作PCB，进行封装集成。

| 名称               | 数量 | 价格  | 用途      |
|------------------|----|-----|---------|
| STM32F103C6T6    | 1  | 8   | 遥控器主控   |
| 摇杆模块             | 2  | 0.5 | 负责输入量获取 |
| 按键模块             | 4  | 0.5 | 负责输入量获取 |
| 无线通信JDY-40       | 1  | 4.5 | 和小车通信   |
| 12864 IIC 中景园    | 1  | 8.5 | 显示遥控器信息 |
| PCB打样 3D打印耗材等杂料费 | 1  | 10  | 外壳，PCB  |
| 动力电池3.7v550ma    | 1  | 15  |         |

小车
名称	数量	价格	用途
SG90舵机	2	5.43	喷头pitch、yaw轴
坦克底盘（走闲鱼二手）	1	36.9	小车底盘
ESP32-CAM	1	27.55	小车视频推流+无线通信
STM32F103C6T6	1	8	底盘主控（72M主频 32k内存）
水泵	1	2.6	喷水
隔离MOS管FR120N	1	1.1	水泵控制
软管	1	1.15	供水
L9110S 双路H桥	1	1.5	驱动底盘电机
GPS	1	12.1	提供位置信息
无线通信JDY-40	1	4.5	遥控器遥控使用
动力电池3.7v2000ma	1	28.99	考虑替换电池。
3D打印耗材等杂料费	*	10	外壳，机械结构等
合计145.25（不包括运费）
遥控器
前期使用模拟通道遥控，后期有精力改用自制遥控器
以下物料清单是自制遥控器的
名称	数量	价格	用途
STM32F103C6T6	1	8	遥控器主控
摇杆模块	2	0.5	负责输入量获取
按键模块	4	0.5	负责输入量获取
无线通信JDY-40	1	4.5	和小车通信
12864 IIC 中景园	1	8.5	显示遥控器信息
PCB打样 3D打印耗材等杂料费	1	10	外壳，PCB
动力电池3.7v550ma	1	15	
合计47（不包括运费）
