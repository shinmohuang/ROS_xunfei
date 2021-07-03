#encoding=UTF-8
import rospy
from xf_mic_tts_offline.srv import *

if __name__ == "__main__":
    rospy.init_node("voiceSuccess")
    client = rospy.ServiceProxy('xf_mic_tts_offline_node/play_txt_wav', Play_TTS_srv)
    response = client.call('您的餐品已送达，请您取餐！','xiaoyan')
    rospy.INFO('语音已发送',response.result)

