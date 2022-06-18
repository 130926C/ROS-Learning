#!  /home/lucks/anaconda3/bin/python

import rospy

if __name__ =="__main__":
    rospy.loginfo("This is param node [Python]")
    # 1.初始化节点
    rospy.init_node("param_node")

    # 1.全局
    rospy.set_param("/param_global", 10)
    # 2.相对
    rospy.set_param("param_relative", 20)
    # 3.私有
    rospy.set_param("~param_private", 30)