#!  /home/lucks/anaconda3/bin/python

import rospy

if __name__ == '__main__':
    rospy.loginfo("This is param op [Python-demo05]")
    # 1.初始化节点
    rospy.init_node("param_op_node")
    # 2.对参数操作
    value = rospy.get_param("heigth")
    rospy.loginfo("Param height is %.2f", value)