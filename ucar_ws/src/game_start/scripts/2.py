#encoding=UTF-8
import rospy
from xf_mic_tts_offline.srv import *
from ht_msg.msg import Ht
longhair=glasses=0
def detectcallback(ht):
    global longhair, glasses
    glasses = ht.glasses_people
    longhair = ht.longhair_people
    print(glasses)
    print(longhair)
    # rospy.signal_shutdown('shut down')

def ht_yy():
    # global longhair, glasses
    rospy.init_node("voiceSuccess")
    rospy.Subscriber("ht_num_info",Ht, detectcallback)
    client = rospy.ServiceProxy('xf_mic_tts_offline_node/play_txt_wav', Play_TTS_srv)
    data = '长头发人数：'+str(longhair)+'人。'+'眼镜个数：'+str(glasses)+'副。'
    response1 = client.call('您的餐品已送达，请您取餐！','xiaoyan')
    response2 = client.call(data,'xiaoyan')
    rospy.INFO('语音已发送',response1.result)
    rospy.INFO('识别已发送',response2.result)

ht_yy()
