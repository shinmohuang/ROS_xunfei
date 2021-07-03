import os
# import threading


def main():
    # add_thread = threading.Thread(target = thread_detect)
    # add_thread.start()
    print('666')
    cmd = ['roslaunch ucar_nav navigation_test.launch',
        'sleep 5; python3 /home/ucar/ROS_xunfei/ucar_ws/src/ht_image/scripts/xunfei2.0.py', 
        'sleep 5; python2 /home/ucar/ROS_xunfei/ucar_ws/src/game_start/scripts/all_in_one.py'
    ]

    open_terminal(cmd)


def open_terminal(commands):
    '''
    打开一个新的终端并执行命令
    未作参数检查，不要试图在传入的命令字符串中添加多余的引号，可能会引发错误
    '''
    cmd_list = []
    for cmd in commands:
        cmd_list.append(""" gnome-terminal --tab -e "bash -c '%s;exec bash'" >/dev/null  2>&1 """ %cmd)

    os.system(';'.join(cmd_list))


# def thread_detect():
#     os.system('python3 /home/ucar/ROS_xunfei/ucar_ws/src/ht_image/scripts/xunfei2.0.py')


if __name__ == "__main__":
    main()