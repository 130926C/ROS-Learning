#! /home/lucks/anaconda3/bin/python

import rospy
from turtlesim.msg import Pose
import time

def dealMsg(pose):
    print("x={:.3f}\ny={:.3f}\ntheta={:.3f}\nangular_velocity={:.3f}\nlinear_velocity={:.3f}".format(pose.x, pose.y, pose.theta, pose.angular_velocity, pose.linear_velocity))
    time.sleep(1)

if __name__ == '__main__':
    rospy.loginfo("This is subscriber [Python]")
    # 1.初始化节点
    rospy.init_node("sub_node")
    # 2.创造订阅者 & 订阅话题
    sub = rospy.Subscriber("/turtle1/pose", Pose, dealMsg, queue_size=10)

    rospy.spin()