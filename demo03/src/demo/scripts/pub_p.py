#! /home/lucks/anaconda3/bin/python

import rospy
from demo.msg import Person

if __name__ == '__main__':
    rospy.loginfo("This is publisher [Python-demo03]")
    # 1.初始化节点
    rospy.init_node("pub_node")
    # 2.创造发布者
    pub = rospy.Publisher("stuinfo", Person, queue_size=10)
    
    # 3.发布信息
    rate = rospy.Rate(1)
    rospy.Duration(2)
    p = Person()
    p.name = "wang5"
    p.id = 0

    while not rospy.is_shutdown():
        pub.publish(p)
        p.id += 1
        rospy.loginfo("Message name=%s, id=%d published by [Pyhton-demo03]", p.name, p.id)
        rate.sleep()
