from bisect import bisect_left
from email import message
import imp
import config as config
from flask import Blueprint, render_template, request, redirect, url_for, jsonify, session, flash
from exts import mail, db
from flask_mail import Message
from models import EmailCaptchaModel, UserModel, ResetCaptchaModel
import string
import random
import threading
import datetime
import time
import blueprints
from blueprints.forms import RegisterForm, LoginForm, ResetForm
from werkzeug.security import generate_password_hash, check_password_hash
from Mqtt_demo.sub import *
from Mqtt_demo.pub import *
from Mqtt_demo.keyboard_control import *
from Mqtt_demo.car_message import *
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

#键盘操作函数

# bt_a=0
# bt_s=0
# bt_d=0
# bt_w=0
# water_pump =0
# power_on =0
# bt_submit =0

@bp.route("/right",methods=['POST'])
def right():
    global bt_d
    bt_d=1
    print(bt_d)
    return jsonify({"code":200})

@bp.route("/left",methods=['POST'])
def left():
    global bt_a
    bt_a=1
    print(f"前进{bt_a}")
    return jsonify({"code":200})
@bp.route("/mforward",methods=['POST'])
def mforward():
    global bt_w
    bt_w=1
    print(bt_w)
    return jsonify({"code":200})
@bp.route("/backward",methods=['POST'])
def backward():
    global bt_s
    bt_s=1
    print(bt_s)
    return jsonify({"code":200})
@bp.route("/water_pumb",methods=['POST'])
def water():
    global water_pump,power_on
    water_pump =0
    power_on =0
    water_pump= not water_pump
    power_on= not power_on
    print(water_pump)
    print(power_on)
    return jsonify({"code":200})
@bp.route("/submit",methods=['POST'])
def submit():
    global bt_submit
    bt_submit=1
    print(bt_submit)
    return jsonify({"code":200})

#电机水源操作函数
@bp.route("/equipment3", methods=['GET', 'POST'])
def equipment3():
    global car_header, car_gps_longitude, car_gps_latitude, \
        car_gps_GTime, car_motor, car_water, car_distance,  \
        car_pitch, car_yaw, car_battery, car_temperature,  \
        car_humidity, car_end, car_errorcode
    global forward_speed,steering_speed,power_on,water_pump,flag_car_pub
    global message_water, message_power
    message_water = "水泵关闭"
    message_power = "电源关闭"
    forward_speed = steering_speed =0
    global bt_w, bt_s, bt_a, bt_d
    #创建进程锁
    # global lock1,lock2
    # lock1= threading.Lock()
    # lock2 = threading.Lock()
    thread_sub =threading.Thread(target=sub_run)
    thread_sub.start()
    thread_pub =threading.Thread(target=pub_run)
    thread_pub.start()
    while(1):
        time.sleep(2)
        
        if request.method == 'GET':
        
            return render_template("equipment3.html")
        else:
            #
            
            
            # 对键盘输入的wasd进行处理，并且能做到控制改变小车速度，返回值为小车forward_speed,steering_speed 
            function_on_speed(bt_w, bt_s, bt_a, bt_d,forward_speed,steering_speed)
            
            # 根据按钮改变水泵与电机状态
            if water_pump == 1:
                water_pump = 0
                message_water = "水泵关闭"
                return render_template("equipment3.html", message_water=message_water)
            elif water_pump == 0:
                water_pump = 1
                message_water = "水泵开启"
                return render_template("equipment3.html", message_water=message_water)

            
            if power_on == 1:
                power_on = 0
                message_water = "电机关闭"
                return render_template("equipment3.html", message_power)
            elif power_on == 0:
                power_on = 1
                message_water = "电机开启"
                return render_template("equipment3.html", message_power)
                #
            #返回操纵电机后，电机的数值
            car_control()

        
            # 重新拆分电机以及水泵的状态
            if car_motor == "OFF_OFF":
                car_motor_car = "OFF"
                car_motor_pumb = "OFF"
            elif car_motor == "OFF_ON":
                car_motor_car = "OFF"
                car_motor_pumb = "ON"
            elif car_motor == "ON_OFF":
                car_motor_car = "ON"
                car_motor_pumb = "OFF"
            else:
                car_motor_car = "ON"
                car_motor_pumb = "ON"
            
            

            # 在右侧中显示输出小车当前状态
            
            if bt_submit == 1:
                # 经度（东/西）E/W
                message_submit_car_gps_longitude = f"小车的经度为：{car_gps_longitude}"
                # 纬度（南/北）S/N
                message_submit_car_gps_latitude = f"小车的纬度为：{car_gps_latitude}"
                # 电机启动（车辆 水泵）
                message_submit_car_motor_car = f"小车开启情况：{car_motor_car}"
                message_submit_car_motor_pumb = f"水泵开启情况：{car_motor_pumb}"
                message_submit_car_water = f"小车剩余水位信息：{car_water}"  # 剩余水位信息
                # 前方距离（cm）
                message_submit_car_distance = f"小车距离前方距离：{car_distance}"
                message_submit_car_pitch = f"云台PItch：{car_pitch}"  # 云台PItch
                message_submit_car_battery = f"云台Yaw：{car_battery}"  # 云台Yaw
                message_submit_car_temperature = f"温度：{car_temperature}"  # 温度
                message_submit_car_humidity = f"湿度：{car_humidity}"  # 湿度
                message_submit_forward_speed = f"小车前进速度：{forward_speed}"
                message_submit_steering_speed = f"小车转向速度：{steering_speed }"
                return render_template("equipment3.html", message_submit_car_gps_longitude=message_submit_car_gps_longitude, message_submit_car_gps_latitude=message_submit_car_gps_latitude,
                                   message_submit_car_motor_car=message_submit_car_motor_car, message_submit_car_water=message_submit_car_water, message_submit_car_distance=message_submit_car_distance, message_submit_car_pitch=message_submit_car_pitch,
                                   message_submit_car_battery=message_submit_car_battery, message_submit_car_temperature=message_submit_car_temperature, message_submit_car_humidity=message_submit_car_humidity,
                                   message_submit_forward_speed=message_submit_forward_speed, message_submit_steering_speed=message_submit_steering_speed, message_submit_car_motor_pumb=message_submit_car_motor_pumb,
                                   )
            
            
            
            
        
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
    return render_template("bounded.html")


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
            user = UserModel(username=username, email=email,
                             password=hash_password, join_time=create_time)
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
        reset_captcha_model = ResetCaptchaModel.query.filter_by(
            email=email).first()
        if reset_captcha_model:
            reset_captcha_model.captcha = captcha
            reset_captcha_model.create_time = datetime.datetime.now()
            db.session.commit()
        else:
            reset_captcha_model = ResetCaptchaModel(
                email=email, captcha=captcha)
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
