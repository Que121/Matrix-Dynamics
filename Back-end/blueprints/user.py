from flask import Blueprint, render_template, request, redirect, url_for, jsonify, session, flash
from exts import mail, db
from flask_mail import Message
from models import EmailCaptchaModel, UserModel
import string
import random
import datetime,time
from .forms import RegisterForm, LoginForm
from werkzeug.security import generate_password_hash, check_password_hash


bp = Blueprint("user", __name__, url_prefix="/user")


@bp.route("/")
def index():
    return render_template("index.html")


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
        email = form.email.data
        password_input = form.password.data
        user_model = UserModel.query.filter_by(email=email).first()
        if user_model:
            if check_password_hash(user_model.password, password=password_input):
                print("登录成功")
                session['user_id'] = user_model.id
                return redirect("/")
            else:
                print("密码输入错误")
                flash("密码输入错误")
                return redirect(url_for("user.login"))
        else:
            print("该用户不存在，请注册")
            flash("该用户不存在，请注册")
            return redirect(url_for("user.register"))

        email = form.email.data
        print(type(EmailCaptchaModel.query.filter_by(email=email).first()))
        print(form.validate())
        # 不知道为什么这个判断格式总是报错
        print("请输入正确格式的账号或密码")
        flash("请输入正确格式的账号或密码")
        return redirect(url_for("user.login"))



@bp.route('/register', methods=['GET', 'POST'])
def register():
    if request.method == 'GET':
        return render_template("register.html")
    else:
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
                return redirect(url_for("user.register"))
            user = UserModel(username=username, email=email, password=hash_password, join_time=create_time)
            db.session.add(user)
            db.session.commit()
            return redirect(url_for("user.login"))
        else:
            print("注册验证失败")
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
        print("captcha_model:",type(captcha_model))
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
