from flask import Blueprint,render_template,request,redirect,url_for,jsonify,session
from exts import mail,db
from flask_mail import Message
from models import EmailCaptchaModel,UserModel
import string
import random
from datetime import datetime
from .forms import RegisterForm,LoginForm
from werkzeug.security import generate_password_hash,check_password_hash

bp = Blueprint("user",__name__,url_prefix="/user")

@bp.route("/login",methods=['GET','POST'])
def login():
    if request.method == 'GET':
        return render_template("login.html")
    else:
        form = LoginForm(request.form)
        if form.validate():
            email = form.email.data
            password = form.password.data
            user = UserModel.query.filter_by(email=email).first()
            if user and check_password_hash(user.password,password):
                session['user_id'] = user.id
                return redirect("/")
            else:

                return redirect(url_for("user.login"))
        else:
            return redirect(url_for("user.login"))

@bp.route("/register", methods=['GET','POST'])
def register():
    if request.method == 'GET':
        return render_template("register.html")
    else:
        form = RegisterForm(request.form)
        if form.validate():
            email = form.email.data
            username = form.username.data
            password = form.password.data

            # md5加密，无法逆向破解可以保护密码安全
            hash_password = generate_password_hash(password)
            user = UserModel(email=email, username=username, password=hash_password)
            db.session.add(user)
            db.session.commit()
            return redirect(url_for("user.login"))
        else:
            return redirect(url_for("user.register"))

@bp.route("/captcha",methods=['POST'])
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
            captcha_model.create_time = datetime.now()
            db.session.commit()
        else:
            captcha_model = EmailCaptchaModel(email=email,captcha=captcha)
            db.session.add(captcha_model)
            db.session.commit()
        # code:200，代表成功的，正常的请求
        return jsonify({"code":200})
    else:
        # code:400，代表客户端错误
        return jsonify({"code":400, "message":"请先传递邮箱！"})

