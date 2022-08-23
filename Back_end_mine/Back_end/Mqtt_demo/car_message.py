from linecache import cache
from nis import match
from operator import index
import random
from socket import MsgFlag
from unittest import case
from paho.mqtt import client as mqtt_client


#小车报文
car_id_ten = 0  # 小车id十位
car_id_bit = 1  # 小车id个位
forward_speed = 0 # 前进速度 0-9 1-5 0停车 1-5前进 6-9后退
steering_speed = 5  # 转向速度 0直走 1-4（慢-快）左转 6-9右转（慢-快）
power_on = 0  # 发1（并且无其它电机操作）解除电机锁 电机上电。
water_pump = 0  # 1开水泵 0关水泵
falg_car_pub = 0

def car_control():
    global forward_speed,steering_speed,power_on,water_pump,falg_car_pub
    if falg_car_pub == 0:
        forward_speed = 0 # 前进速度 0-9 1-5 0停车 1-5前进 6-9后退
        steering_speed = 5  # 转向速度 0直走 1-4（慢-快）左转 6-9右转（慢-快）
        power_on = 0  # 发1（并且无其它电机操作）解除电机锁 电机上电。
        water_pump = 0  # 1开水泵 0关水泵
        falg_car_pub = falg_car_pub + 1
        return(forward_speed,steering_speed,power_on,water_pump,falg_car_pub)
    else:
        forward_speed = forward_speed  # 前进速度 0-9 1-5 0停车 1-5前进 6-9后退
        steering_speed = steering_speed  # 转向速度 0直走 1-4（慢-快）左转 6-9右转（慢-快）
        power_on = power_on  # 发1（并且无其它电机操作）解除电机锁 电机上电。
        water_pump = water_pump  # 1开水泵 0关水泵
        falg_car_pub = falg_car_pub + 1
        return(forward_speed,steering_speed,power_on,water_pump,falg_car_pub)



