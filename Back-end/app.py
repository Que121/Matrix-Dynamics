from flask import Flask, session, g
from flask_migrate import Migrate
import config
from exts import db, mail
# from blueprints.qa import bp as qa_bp
from blueprints import qa_bp
from blueprints import user_bp
from gevent import pywsgi
from models import UserModel

app = Flask(__name__)
app.config.from_object(config)
db.init_app(app)
mail.init_app(app)
migrate = Migrate(app, db)

app.register_blueprint(qa_bp)
app.register_blueprint(user_bp)


# 钩子函数
@app.before_request
def before_request():
    user_id = session.get("user_id")
    if user_id:
        try:
            user = UserModel.query.get(user_id)
            # 全局变量，单次请求的时候会有一个全局变量g
            # 给g绑定一个user的变量，它的值是user这个变量
            # setattr(g,"user",user)
            g.user = user
        except:
            g.user = None

# 请求来了，先执行before_request再执行视图函数，视图函数中返回模板再执行context_processor
# 上下文处理器
@app.context_processor
def context_processor():
    if hasattr(g,"user"):
        return {"user": g.user}
    else:
        return {}


if __name__ == '__main__':
    server = pywsgi.WSGIServer(('0.0.0.0', 5000), app)
    server.serve_forever()
    app.run()
