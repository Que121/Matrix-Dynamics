from flask import Flask
from flask_migrate import Migrate
import config
from exts import db,mail
# from blueprints.qa import bp as qa_bp
from blueprints import qa_bp
from blueprints import user_bp
from gevent import pywsgi


app = Flask(__name__)
app.config.from_object(config)
db.init_app(app)
mail.init_app(app)
migrate = Migrate(app, db)

app.register_blueprint(qa_bp)
app.register_blueprint(user_bp)

if __name__ == '__main__':
    server = pywsgi.WSGIServer(('0.0.0.0', 5000), app)
    server.serve_forever()
    app.run()
