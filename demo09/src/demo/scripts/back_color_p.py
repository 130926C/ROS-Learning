#! /home/lucks/anaconda3/bin/python


import rospy
from turtlesim.msg import Color
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

if __name__ == '__main__':
    rospy.loginfo("This is back color node")
    # 1.初始化节点
    rospy.init_node("color_node")
    # 2.修改参数
    rospy.set_param("/turtlesim/background_b", 125)
    rospy.set_param("/turtlesim/background_c", 125)
    rospy.set_param("/turtlesim/background_c", 125)

    # 3.动态刷新
    # (1) 创建client
    client = rospy.ServiceProxy("/clear", Empty)    # python在这里是有坑的，不能写成 EmptyRequest
    # (2) 创建发送的请求
    empty = EmptyRequest()
    # (3) 发送请求
    client.wait_for_service()
    client.call(empty)

