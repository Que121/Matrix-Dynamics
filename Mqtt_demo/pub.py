import random
import time

from paho.mqtt import client as mqtt_client


# 设置 MQTT Broker 连接参数
broker = '43.138.132.49'
port = 1883
topic = "car_control"
# generate client ID with pub prefix randomly
client_id = f'python-mqtt-{random.randint(0, 1000)}'

# 小车控制报文定义
frame_header = "CTR"  # 头帧
car_id_ten = 0  # 小车id十位
car_id_bit = 1  # 小车id个位
forward_speed = 5  # 前进速度 0-9 1-5 0停车 1-5前进 6-9后退
steering_speed = 4  # 转向速度 0直走 1-4（慢-快）左转 6-9右转（慢-快）
power_on = 1  # 发1（并且无其它电机操作）解除电机锁 电机上电。
water_pump = 0  # 1开水泵 0关水泵
frame_end = "\n"  # 帧尾


# MQTT连接函数
def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client

# 发布消息
def publish(client):  
    while True:
        time.sleep(2)
        msg = f"{frame_header}{car_id_ten}{car_id_bit}{forward_speed}{steering_speed}{power_on}{water_pump}{frame_end}"  # 待发布的消息
        result = client.publish(topic, msg)
        # result: [0, 1]
        status = result[0]
        if status == 0:
            print(f"发给小车的控制报文为：{msg}")
        else:
            print(f"Failed to send message to topic {topic}")
        


def run():
    client = connect_mqtt()
    client.loop_start()
    publish(client)


if __name__ == '__main__':
    run()
