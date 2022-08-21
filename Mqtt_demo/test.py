from pub import pub_run
from sub import sub_run
import time
import threading


def main():
    while (1):
        time.sleep(3)
        print("!!!!!!!!!!!")


thread_sub = threading.Thread(target=sub_run)
thread_sub.start()

thread_pub = threading.Thread(target=pub_run)
thread_pub.start()

thread_main = threading.Thread(target=main)
thread_main.start()
