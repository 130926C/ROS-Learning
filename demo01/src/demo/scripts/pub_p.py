#! /home/lucks/anaconda3/bin/python

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.loginfo("This is publisher [Python]")
    # 1.初始化节点
    rospy.init_node("pub_node")
    # 2.创建发布者 & 注册话题
    pub = rospy.Publisher("hello", String, queue_size=10)
    # 3.发布信息
    rate = rospy.Rate(1)
    rospy.Duration(2)
    msg = String()
    msg = "Hello, this is publisher from demo01 send by [Python]"
    while not rospy.is_shutdown():
        pub.publish(msg)
        rospy.loginfo("Messgae: [%s] published. [Python]", msg)
        rate.sleep()
            

