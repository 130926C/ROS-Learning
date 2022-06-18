#! /home/lucks/anaconda3/bin/python

import rospy
from demo.srv import AddNumRequest, AddNumResponse, AddNum

def dealRequest(req):
    rospy.loginfo("Recevied Request [%d,%d], Response is [%d]", req.num1, req.num2, req.num1+req.num2)
    resp = AddNumResponse()
    resp.sum = req.num1 + req.num2
    return resp


if __name__ == '__main__':
    rospy.loginfo("This is server [Python-demo04]")
    # 1.初始化节点
    rospy.init_node("server_node")
    # 2.创建服务器 & 注册话题
    server = rospy.Service("addNum", AddNum, dealRequest)
    # 3.spin
    rospy.spin()