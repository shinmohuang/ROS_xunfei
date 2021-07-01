import os
import threading
def main():
    add_thread = threading.Thread(target = thread_detect)
    add_thread.start()
    print('666')

def thread_detect():
    os.system('python3 /home/ucar/ROS_xunfei/ucar_ws/src/ht_image/scripts/xunfei2.0.py')


if __name__ == "__main__":
    main()