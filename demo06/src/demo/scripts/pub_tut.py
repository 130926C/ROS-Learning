#! /home/lucks/anaconda3/bin/python

import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.loginfo("This is control node [Python]")
    # 1.初始化节点
    rospy.init_node("pub_node")
    # 2.创建发布者 & 订阅话题   
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist)
    # 3.发布信息
    rate = rospy.Rate(1)
    rospy.Duration(2)

    geT = Twist()
    geT.linear.x = 2.0
    geT.angular.z = 0.5

    while not rospy.is_shutdown():
        pub.publish(geT)
        rospy.loginfo("Control Message Send successed")
        rate.sleep()
