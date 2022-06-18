#! /home/lucks/anaconda3/bin/python

import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.loginfo("This is topic node [Python]")
    # 1.初始化节点 
    rospy.init_node("topic_node")
    
    # A.全局
    pub_g = rospy.Publisher("/topic_global",String, queue_size=10)
    # B.相对
    pub_r = rospy.Publisher("topic_relateive", String, queue_size=10)
    # C.私有
    pub_p = rospy.Publisher("~topic_private", String, queue_size=10)