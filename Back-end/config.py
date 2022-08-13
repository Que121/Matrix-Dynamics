# 数据库的配置变量
HOSTNAME = '127.0.0.1'
PORT     = '3306'
DATABASE = 'user'
USERNAME = 'root'
PASSWORD = 'kevinabc2002'
DB_URI = 'mysql+pymysql://{}:{}@{}:{}/{}?charset=utf8'.format(USERNAME,PASSWORD,HOSTNAME,PORT,DATABASE)
SQLALCHEMY_DATABASE_URI = DB_URI
SQLALCHEMY_TRACK_MODIFICATIONS = True
SECRET_KEY = "kevinabc2002"

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
