from linecache import cache
from nis import match
from operator import index
import random
from socket import MsgFlag
from unittest import case

from paho.mqtt import client as mqtt_client

# 设置 MQTT Broker 连接参数
broker = '43.138.132.49'
port = 1883
topic = "car_state"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 100)}'
msg = "default"
car_state = "ID:01_GPS:E11919.42N2605.30GTime:00:00:00_OFF_OFF_Wt:000_D:0.00_P:0.0Y:0.0B:000_T:31.7_H:49.0_Ready"
car_state = "ID:01_GPS:E11919.42N2605.30GTime:00:00:00_OFF_OFF_Wt:000_D:0.00_P:0.0Y:0.0B:000_T:31.7_H:49.0_Error1"
# MQTT连接函数


def connect_mqtt() -> mqtt_client:
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


# 订阅消息
def subscribe(client: mqtt_client):
    def on_message(client, userdata, msg):
        global car_state
        car_state = msg.payload.decode()  # 接收
        print(f"{msg.payload.decode()}")
        decode()
        print("\n")
    client.subscribe(topic)
    client.on_message = on_message


# 小车回报报文解码定义
car_header = 0  # 帧头
car_gps_longitude = 0  # 经度（东/西）E/W
car_gps_latitude = 0  # 纬度（南/北）S/N
car_motor = 0  # 电机启动（车辆 水泵）
car_water = 0  # 剩余水位信息
car_distance = 0  # 前方距离（cm）
car_pitch = 0  # 云台PItch
car_battery = 0  # 云台Yaw
car_temperature = 0  # 温度
car_humidity = 0  # 湿度
car_end = 0  # 帧尾
car_errorcode = 0  # 故障代码


# 解码
def decode():
    global car_header, car_gps_longitude, car_gps_latitude, \
        car_gps_GTime, car_motor, car_water, car_distance,  \
        car_pitch, car_yaw, car_battery, car_temperature,  \
        car_humidity, car_end, car_errorcode
# =======================================================================================
    for index_header in range(len(car_state[:(len(car_state)-5)])):
        if ((f"{car_state[index_header]}{car_state[index_header+1]}") == "ID"):
            for index_end in range(len(car_state[index_header:(len(car_state)-5)])):
                if ((f"{car_state[index_end]}{car_state[index_end+1]}") == "GP"):
                    car_header = car_state[index_header+3:index_end-1]
# =======================================================================================
        elif ((f"{car_state[index_header]}") == "E"):
            for index_end in range(len(car_state[index_header:(len(car_state)-5)])):
                if ((f"{car_state[index_end]}") == "N"):
                    car_gps_longitude = car_state[index_header:index_end]
# =======================================================================================
        elif ((f"{car_state[index_header]}") == "N"):
            for index_end in range(len(car_state[index_header:(len(car_state)-5)])):
                if ((f"{car_state[index_end]}{car_state[index_end+1]}") == "GT"):
                    car_gps_latitude = car_state[index_header:index_end]
# =======================================================================================
        elif ((f"{car_state[index_header]}{car_state[index_header+1]}") == "GT"):
            car_gps_GTime = car_state[index_header+6:index_header+14]
# =======================================================================================
        elif ((f"{car_state[index_header]}{car_state[index_header+1]}") == "me"):
            for index_end in range(len(car_state[index_header:(len(car_state)-5)])):
                if ((f"{car_state[index_end]}{car_state[index_end+1]}") == "Wt"):
                    car_motor = car_state[index_header+12:index_end-1]
# =======================================================================================
        elif ((f"{car_state[index_header]}{car_state[index_header+1]}") == "Wt"):
            car_water = car_state[index_header+3:index_header+6]
# =======================================================================================
        elif ((f"{car_state[index_header]}") == "D"):
            car_distance = car_state[index_header+2:index_header+6]
# =======================================================================================
        elif ((f"{car_state[index_header]}") == "P"):
            car_pitch = car_state[index_header+2:index_header+5]
# =======================================================================================
        elif ((f"{car_state[index_header]}") == "Y"):
            car_yaw = car_state[index_header+2:index_header+5]
# =======================================================================================
        elif ((f"{car_state[index_header]}") == "B"):
            car_battery = car_state[index_header+2:index_header+5]
# =======================================================================================
        elif ((f"{car_state[index_header]}") == "T"):
            car_temperature = car_state[index_header+2:index_header+6]
# =======================================================================================
        elif ((f"{car_state[index_header]}") == "H"):
            car_humidity = car_state[index_header+2:index_header+6]
# =======================================================================================
    car_end = car_state[(len(car_state)-6):(len(car_state)-1)]
    car_errorcode = car_state[(len(car_state)-1)]
    for index_header in range(len(car_state)):
        if ((f"{car_state[index_header]}") == "R"):
            car_end = car_state[(len(car_state)-5):]
            car_errorcode = 0
# =======================================================================================
    print(f"{car_header}\n{car_gps_longitude}\n{car_gps_latitude} \
      \n{car_gps_GTime}\n{car_motor}\n{car_water} \
      \n{car_distance}\n{car_pitch}\n{car_yaw} \
      \n{car_battery}\n{car_temperature}\n{car_humidity}\n{car_end} \
      \n{car_errorcode}")


def run():
    client = connect_mqtt()
    subscribe(client)
    client.loop_forever()


if __name__ == '__main__':
    # run()
    decode()
