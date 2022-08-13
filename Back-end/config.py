# 数据库的配置变量
HOSTNAME = '10.0.8.12'
PORT     = '3306'
DATABASE = 'user'
USERNAME = 'root'
PASSWORD = 'Qh13005968844'
DB_URI = 'mysql+pymysql://{}:{}@{}:{}/{}?charset=utf8'.format(USERNAME,PASSWORD,HOSTNAME,PORT,DATABASE)
SQLALCHEMY_DATABASE_URI = DB_URI
SQLALCHEMY_TRACK_MODIFICATIONS = True
# SECRET_KEY = "020128"

# 邮箱配置
# 项目中用的是QQ邮箱
MAIL_SERVER = "smtp.qq.com"
MAIL_PORT = 465
MAIL_USE_TLS = False
MAIL_USE_SSL = True
MAIL_DEBUG = True
MAIL_USERNAME = "1409182558@qq.com"
MAIL_PASSWORD = "bwaoppfmvtqwgege"
MAIL_DEFAULT_SENDER = "1409182558@qq.com"
    