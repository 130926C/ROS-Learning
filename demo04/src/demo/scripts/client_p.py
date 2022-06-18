#! /home/lucks/anaconda3/bin/python

import rospy
from demo.srv import AddNumRequest, AddNumResponse, AddNum
import sys

if __name__ == '__main__':
    rospy.loginfo("This is Client [Python-demo04]")
    if len(sys.argv) == 1:
        rospy.loginfo("NO number input, use default [10, -3]")
    elif len(sys.argv) == 3:
        rospy.loginfo("Detect input number is [%d, %d]", sys.argv[1], sys.argv[2])
    else:
        rospy.loginfo("Input number count is not TWO")
        sys.exit(1)

    # 1.初始化节点
    rospy.init_node("client_node")
    # 2.创建客户端 & 关注话题
    client = rospy.ServiceProxy("addNum", AddNum)
    # 3.发送请求
    client.wait_for_service()
    req = AddNumRequest()
    if len(sys.argv) == 1:
        req.num1 = 10
        req.num2 = -3
    elif len(sys.argv) == 3:
        req.num1 = int(sys.argv[1])
        req.num2 = int(sys.argv[2])

    resp = client.call(req)
    rospy.loginfo("Received response is [%d]", resp.sum)