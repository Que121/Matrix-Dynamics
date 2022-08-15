import wtforms
from wtforms.validators import length, email, EqualTo, InputRequired
from models import EmailCaptchaModel, UserModel


class LoginForm(wtforms.Form):
    email = wtforms.StringField(validators=[email()])
    password = wtforms.StringField(validators=[length(min=6, max=20)])


class RegisterForm(wtforms.Form):
    username = wtforms.StringField(validators=[length(min=3, max=20, message="长度在3和20之间")])
    email = wtforms.StringField(validators=[email()])
    captcha = wtforms.StringField(validators=[length(min=4, max=4)])
    password = wtforms.StringField(validators=[length(min=6, max=20, message="长度在6和20之间")])
    password_confirm = wtforms.StringField(validators=[EqualTo("password")])

    def validate_captcha(self, field):
        captcha = field.data
        email = self.email.data
        captcha_model = EmailCaptchaModel.query.filter_by(email=email).first()
        print(captcha_model.captcha)
        if not captcha_model or captcha_model.captcha.lower() != captcha.lower():
            print("验证码错误")
            raise wtforms.ValidationError("邮箱验证码错误！")

    def validate_email(self, field):
        email = field.data
        user_model = UserModel.query.filter_by(email=email).first()
        if user_model:
            print("邮箱已存在")
            raise wtforms.ValidationError("邮箱已经存在！")


class QuestionForm(wtforms.Form):
    title = wtforms.StringField(validators=[length(min=3, max=200)])
    content = wtforms.StringField(validators=[length(min=5)])


class AnswerForm(wtforms.Form):
    content = wtforms.StringField(validators=[length(min=1)])