#! /home/lucks/anaconda3/bin/python

import rospy
from std_msgs.msg import String

def dealMsg(data):
    rospy.loginfo("Received Messages is %s", data.data)

if __name__ == '__main__':
    rospy.loginfo("This is subscriber [Python-demo02]")
    # 1.初始化节点
    rospy.init_node("sub_node")
    # 2.创建订阅者 & 关注话题   
    sub = rospy.Subscriber("hello", String, dealMsg)
    # 3.spin
    rospy.spin()