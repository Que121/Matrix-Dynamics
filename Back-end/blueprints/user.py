from flask import Blueprint, render_template, request, redirect, url_for, jsonify, session, flash, g
from decorators import login_required
from exts import mail, db
from flask_mail import Message
from models import EmailCaptchaModel, UserModel, ResetCaptchaModel
import string
import random
import datetime, time
import blueprints
from blueprints.forms import RegisterForm, LoginForm, ResetForm
from werkzeug.security import generate_password_hash, check_password_hash

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


@bp.route("equipment3")
def equipment3():
    message_water = "水泵关闭"
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
                           message_submit_car_motor_car=message_submit_car_motor_car)


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
