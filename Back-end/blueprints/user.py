from flask import Blueprint, render_template, request, redirect, url_for, jsonify, session, flash, g
from decorators import login_required
from exts import mail, db
from flask_mail import Message
from models import EmailCaptchaModel, UserModel, ResetCaptchaModel
import string
import random
import datetime
import time
import blueprints
from blueprints.forms import RegisterForm, LoginForm, ResetForm
from werkzeug.security import generate_password_hash, check_password_hash
from Mqtt_demo.keyboard_control import *
from Mqtt_demo.car_message import *
import threading
from paho.mqtt import client as mqtt_client
from Mqtt_demo.car_message import *
import random
import time
from turtle import forward
import sys
import os
from linecache import cache
from nis import match
from operator import index
import random
from socket import MsgFlag
from unittest import case


# 设置 MQTT Broker 连接参数 pub
broker_pub = '43.138.132.49'
port_pub = 1883
topic_pub = "car_control"
# generate client ID with pub prefix randomly
client_id_pub = f'python-mqtt-{random.randint(0, 1000)}'


# 设置 MQTT Broker 连接参数 sub
broker_sub = '43.138.132.49'
port_sub = 1883
topic_sub = "car_state"
# generate client ID with pub prefix randomly
client_id_sub = f'python-mqtt-{random.randint(0, 100)}'
msg_sub = "default"
car_state = "ID:01_GPS:E11919.42N2605.30GTime:00:00:00_OFF_OFF_Wt:000_D:0.00_P:0.0Y:0.0B:000_T:31.7_H:49.0_Ready"

global car_header, car_gps_longitude, car_gps_latitude, \
    car_gps_GTime, car_motor, car_water, car_distance,  \
    car_pitch, car_yaw, car_battery, car_temperature,  \
    car_humidity, car_end, car_errorcode
global mauto
car_header = 0
car_gps_longitude = 0
car_gps_latitude = 0
car_gps_GTime = 0
car_motor = 0
car_water = 0
car_distance = 0
car_pitch = 0
car_yaw = 0
car_battery = 0
car_temperature = 0
car_humidity = 0
car_end = 0
car_errorcode = 0
flash_flag = 0
# 小车控制报文定义
frame_header = "CTR"  # 帧头
car_id_ten = 0  # 小车id十位
car_id_bit = 1  # 小车id个位
frame_end = "\n"  # 帧尾

mforward = 0
mleft = 0
mauto = 0
StringAuto = '自动巡航模式关闭'
water_pump = 0
bp = Blueprint("user", __name__, url_prefix="/user")


@bp.route("/equipment")
def equipment():
    return render_template("equipment.html")


@bp.route("/equipment1_1")
def equipment1_1():
    return render_template("equipment1_1.html")


@bp.route("/equipment1_2")
def equipment1_2():
    return render_template("equipment1_2.html")


@bp.route("/forward", methods=['POST'])
def forward():
    global steering_speed, forward_speed, power_on, water_pump
    power_on = 1  # 发1（并且无其它电机操作）解除电机锁 电机上电。
    # water_pump = 1  # 1开水泵 0关水泵
    if (forward_speed > 5):
        forward_speed -= forward_speed
    elif (forward_speed == 5):
        forward_speed = 5
    elif (forward_speed >= 0 and forward_speed < 5):
        forward_speed += 1
    else:
        forward_speed = 0
    print(f"前进速度：{forward_speed}")
    return jsonify({"code": 200})


@bp.route("/left", methods=['POST'])
def left():
    #值1-4是左转
    global steering_speed, forward_speed, power_on, water_pump
    power_on = 1  # 发1（并且无其它电机操作）解除电机锁 电机上电。
    # water_pump = 1  # 1开水泵 0关水泵
    if (steering_speed == 5):
        steering_speed = 0
    if (steering_speed >= 0 and steering_speed < 5):
        steering_speed += 1
    if (steering_speed < 0):
        steering_speed = 0
    if (steering_speed >= 5 and steering_speed < 10):
        steering_speed -= 1
    print(f"转向速度：{steering_speed}")
    return jsonify({"code": 200})


@bp.route("/backward", methods=['POST'])
def backward():
    global steering_speed, forward_speed, power_on, water_pump
    power_on = 1  # 发1（并且无其它电机操作）解除电机锁 电机上电。
    # water_pump = 1  # 1开水泵 0关水泵
    if (forward_speed >= 0 and forward_speed < 5):
        forward_speed -= 1
    elif (forward_speed == 5):
        forward_speed = 4
    if (forward_speed == -1):
        forward_speed = 5
    if (forward_speed >= 5):
        forward_speed = forward_speed + 1

    print(f"前进速度：{forward_speed}")
    return jsonify({"code": 200})


@bp.route("/right", methods=['POST'])
def right():
    global steering_speed, forward_speed, power_on, water_pump
    power_on = 1  # 发1（并且无其它电机操作）解除电机锁 电机上电。
    # water_pump = 1  # 1开水泵 0关水泵
    if (steering_speed == 0):
        steering_speed = 5
    if (steering_speed >= 5 and steering_speed < 9):
        steering_speed += 1
    if (steering_speed < 5 and steering_speed > 0):
        steering_speed -= 1
    print(f"转向速度：{steering_speed}")
    return jsonify({"code": 200})


@bp.route("/fwater", methods=['POST'])
def fwater():
    global steering_speed, forward_speed, power_on, water_pump
    power_on = 1  # 发1（并且无其它电机操作）解除电机锁 电机上电。
    water_pump = not water_pump  # 1开水泵 0关水泵
    print(f"浇水开关：{water_pump}")
    return jsonify({"code": 200})


@bp.route("/auto", methods=['POST'])
def auto():
    global mauto
    global StringAuto
    mauto += 1
    StringAuto = "自动巡航模式开启"
    if mauto == 2:
        print("自动巡航模式关闭")
        StringAuto = "自动巡航模式关闭"
        mauto = 0
    print("自动巡航模式")
    return jsonify({"code": 200})

# MQTT连接函数 pub


def connect_mqtt():
    def on_connect(client_pub, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker pub!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client_pub = mqtt_client.Client(client_id_pub)
    client_pub.on_connect = on_connect
    client_pub.connect(broker_pub, port_pub)
    return client_pub

# MQTT连接函数 sub


def connect_mqtt_sub() -> mqtt_client:
    def on_connect(client_sub, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker sub!")
        else:
            print("SUB Failed to connect, return code %d\n", rc)

    client_sub = mqtt_client.Client(client_id_sub)
    client_sub.on_connect = on_connect
    client_sub.connect(broker_sub, port_sub)
    return client_sub

# 发布消息


def publish(client_pub):
    while True:
        time.sleep(2)
        # 直接在函数里用
        # 待发布的消息
        msg_pub = f"{frame_header}{car_id_ten}{car_id_bit}{forward_speed}{steering_speed}{power_on}{water_pump}{frame_end}"
        result = client_pub.publish(topic_pub, msg_pub)
        # result: [0, 1]
        status = result[0]
        if status == 0:
            print(f"发给小车的控制报文为：{msg_pub}")
        else:
            print(f"Failed to send message to topic pub {topic_pub}")

# 发送报文


def pub_run():
    client_pub = connect_mqtt()
    client_pub.loop_start()
    publish(client_pub)

# 订阅消息


def subscribe(client_sub: mqtt_client):
    def on_message(client_sub, userdata, msg_sub):
        global car_state
        car_state = msg_sub.payload.decode()  # 接收
        print(f"{msg_sub.payload.decode()}")
        decode()
        print("\n")
    client_sub.subscribe(topic_sub)
    client_sub.on_message = on_message

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


def sub_run():
    client_sub = connect_mqtt_sub()
    subscribe(client_sub)
    client_sub.loop_forever()


@bp.route("equipment3", methods=['GET', 'POST'])
def equipment3():
    thread_sub = threading.Thread(target=sub_run)
    thread_sub.start()
    thread_pub = threading.Thread(target=pub_run)
    thread_pub.start()
    message_water = "水泵开启"
    message_power = "电源关闭"
    # 经度（东/西）E/W
    message_submit_car_gps_longitude = f"小车的经度为：{1}"
    # 纬度（南/北）S/N
    message_submit_car_gps_latitude = f"小车的纬度为：{1}"
    message_submit_car_motor_car = f"小车开启情况：{1}"  # 电机启动（车辆 水泵）
    message_submit_car_motor_pumb = f"水泵开启情况：{1}"
    message_submit_car_water = f"小车剩余水位信息：{1}"  # 剩余水位信息
    message_submit_car_distance = f"小车距离前方距离：{1}"  # 前方距离（cm）
    message_submit_car_pitch = f"云台PItch：{1}"  # 云台PItch
    message_submit_car_battery = f"云台Yaw：{1}"  # 云台Yaw
    message_submit_car_temperature = f"温度：{1}"  # 温度
    message_submit_car_humidity = f"湿度：{1}"  # 湿度
    message_submit_forward_speed = f"小车前进速度：{1}"
    message_submit_steering_speed = f"小车转向速度：{1}"
    return render_template("equipment3.html", message_submit_car_gps_longitude=message_submit_car_gps_longitude,
                           message_submit_car_gps_latitude=message_submit_car_gps_latitude,
                           message_submit_car_motor_car=message_submit_car_motor_car,
                           message_submit_car_motor_pumb=message_submit_car_motor_pumb,
                           message_submit_car_water=message_submit_car_water,
                           message_submit_car_distance=message_submit_car_distance,
                           message_submit_car_pitch=message_submit_car_pitch,
                           message_submit_car_battery=message_submit_car_battery,
                           message_submit_car_temperature=message_submit_car_temperature,
                           message_submit_car_humidity=message_submit_car_humidity,
                           message_submit_forward_speed=message_submit_forward_speed,
                           message_submit_steering_speed=message_submit_steering_speed,
                           mforward=mforward,
                           StringAuto=StringAuto)

@bp.route("/success")
def success():
    return render_template("success.html")


@bp.route("/success2")
def success2():
    return render_template("success2.html")


@bp.route("/bounded2")
def bounded2():
    return render_template("bounded2.html")


@bp.route("/bounded3")
def bounded3():
    return render_template("bounded3.html")


@bp.route("/bounded4")
def bounded4():
    return render_template("bounded4.html")


@bp.route("/")
def index():
    return render_template("index.html")


@bp.route("/mainindex")
def mainindex():
    return render_template("mainindex.html")


@bp.route("/bounded")
def bounded():
    # 得先判断用户是否已经注册，如果没有登录就跳转到登录页面
    if hasattr(g, 'user'):
        return render_template("bounded.html")
    else:
        return render_template("login.html", message="请先进行登录")


@bp.route("/team")
def team():
    return render_template("team.html")


@bp.route("/purchase")
def purchase():
    return render_template("purchase.html")


@bp.route('/login', methods=['GET', 'POST'])
def login():
    """登录：guest1:123456
    0.通过验证
    1.通过邮箱查找出user_model
    2.如果存在就比较密码是否正确 正确：登录成功 不正确：密码错误
    3.不存在直接提示用户不存在并返回到注册页面"""
    if request.method == 'GET':
        return render_template("login.html")
    else:
        form = LoginForm(request.form)
        if form.validate():
            email = form.email.data
            password_input = form.password.data
            user_model = UserModel.query.filter_by(email=email).first()
            if user_model:
                if check_password_hash(user_model.password, password=password_input):
                    print("登录成功")
                    session['user_id'] = user_model.id
                    if hasattr(g, "user"):
                        print(g.user.username)
                    return redirect(url_for("user.success"))
                else:
                    print("密码输入错误")
                    flash("密码输入错误")
                    return redirect(url_for("user.login"))
            else:
                print("该用户不存在，请注册")
                flash("该用户不存在，请注册")
                return redirect(url_for("user.login"))
        else:
            print("请输入正确格式的账号或密码")
            flash("请输入正确格式的账号或密码")
            return redirect(url_for("user.login"))


@bp.route("/mine")
def mine():
    return render_template("mine.html")


@bp.route("/logout")
def logout():
    # 清除session中的所有数据
    session.clear()
    return redirect(url_for('user.login'))


@bp.route('/register', methods=['GET', 'POST'])
def register():
    if request.method == 'GET':
        return render_template("register.html")
    else:
        print("POST方法进行中")
        form = RegisterForm(request.form)
        if form.validate():
            print("验证成功")
            username = form.username.data
            email = form.email.data
            password = form.password.data
            # 密码加密
            hash_password = generate_password_hash(password=password)
            captcha = form.captcha.data
            create_time = datetime.datetime.now()
            # 1.通过email查询user表 如果存在就通知已存在该用户 不存在就新建
            user_model = UserModel.query.filter_by(email=email).first()
            print(type(user_model))
            if user_model:
                print("该邮箱已被注册，请重新输入")
                flash("该邮箱已被注册，请重新输入")
                return redirect(url_for("user.register"))
            user = UserModel(username=username, email=email, password=hash_password, join_time=create_time)
            db.session.add(user)
            db.session.commit()
            return redirect(url_for("user.success2"))
        else:
            print("注册验证失败")
            flash("注册验证失败")
            return redirect(url_for("user.register"))


@bp.route("/captcha", methods=['POST'])
def get_captcha():
    # 然用户自己数输入邮箱有两种方式一种是GET请求，一种是POST请求
    # POST方法用form来获取
    email = request.form.get("email")
    letters = string.ascii_letters + string.digits
    captcha = "".join(random.sample(letters, 4))
    if email:
        message = Message(
            subject="矩阵动力",
            recipients=[email],
            body=f"矩阵动力提示您，您的注册验证码是：{captcha}，请不要将此验证码告诉其他人"
        )
        mail.send(message)
        captcha_model = EmailCaptchaModel.query.filter_by(email=email).first()
        if captcha_model:
            captcha_model.captcha = captcha
            captcha_model.create_time = datetime.datetime.now()
            db.session.commit()
        else:
            captcha_model = EmailCaptchaModel(email=email, captcha=captcha)
            db.session.add(captcha_model)
            db.session.commit()
        print("captcha:", captcha)
        # code:200，代表成功的，正常的请求
        return jsonify({"code": 200})
    else:
        # code:400，代表客户端错误
        return jsonify({"code": 400, "message": "请先传递邮箱！"})


@bp.route("/mail", methods=['POST'])
def my_mail():
    email = request.form.get("email")
    letters = string.ascii_letters + string.digits
    captcha = "".join(random.sample(letters, 4))

    if email:
        message = Message(
            subject="矩阵动力",
            recipients=[email],
            body=f"您的验证码是{captcha}只能用于验证找回操作，请不要告诉任何人"
        )
        mail.send(message)
        reset_captcha_model = ResetCaptchaModel.query.filter_by(email=email).first()
        if reset_captcha_model:
            reset_captcha_model.captcha = captcha
            reset_captcha_model.create_time = datetime.datetime.now()
            db.session.commit()
        else:
            reset_captcha_model = ResetCaptchaModel(email=email, captcha=captcha)
            db.session.add(reset_captcha_model)
            db.session.commit()
        print("captcha", captcha)
        return jsonify({"code": 200})
    else:
        return jsonify({"code": 400, "message": "请先传递邮箱！"})


@bp.route("/reset", methods=['GET', 'POST'])
def reset():
    if request.method == 'GET':
        return render_template("password.html")
    else:
        print("111POST方法")
        form = ResetForm(request.form)
        if form.validate():
            email = form.email.data
            captcha = form.captcha.data
            password = form.password.data
            user = ResetCaptchaModel(email=email)
            hash_password = generate_password_hash(password=password)
            user_model = UserModel.query.filter_by(email=email).first()
            # 如果查询到该邮箱存在执行重新修改密码的操作
            if user_model:
                print("确实是已有账户")
                user_model.password = hash_password
                db.session.commit()
                print("修改成功")
                return redirect(url_for("user.resetsuccess"))
            else:
                print("请注册新的账户")
                flash("请注册新的账户")
                return redirect(url_for("user.reset"))
        else:
            print("修改失败")
            flash("注册失败，您的输入有问题")
            return redirect(url_for("user.reset"))


@bp.route("/resetsuccess")
def resetsuccess():
    return render_template("resetsuccess.html")
