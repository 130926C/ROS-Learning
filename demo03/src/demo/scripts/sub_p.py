#! /home/lucks/anaconda3/bin/python

import rospy
from demo.msg import Person

def dealMsg(p):
    rospy.loginfo("Received Message: name=%s, id=%d, deal by [Python-demo03]", p.name, p.id)

if __name__ == '__main__':
    rospy.loginfo("This is subscriber [Python-demo03]")
    # 1.初始化节点
    rospy.init_node("sub_node")
    # 2.创造订阅者 & 关注话题
    sub = rospy.Subscriber("stuinfo", Person, dealMsg)
    # 3.spin
    rospy.spin()