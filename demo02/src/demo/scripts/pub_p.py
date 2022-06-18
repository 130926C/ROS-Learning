#! /home/lucks/anaconda3/bin/python

import rospy
from std_msgs.msg import String


if __name__ == '__main__':
    rospy.loginfo("This is publisher [Python-demo02]")
    # 1.初始化节点
    rospy.init_node("pub_node")
    # 2.创建发布者
    pub = rospy.Publisher("hello", String, queue_size=10)
    # 3.发布信息
    rate = rospy.Rate(1)
    rospy.Duration(2)
    msg = String()
    curIndex = 0

    while not rospy.is_shutdown():
        msg.data = str("Hello, Message from [Python-demo02] with index={}".format(curIndex))
        curIndex+=1
        pub.publish(msg)
        rospy.loginfo("Published Message is %s", msg.data)
        rate.sleep()
