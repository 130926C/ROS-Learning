#! /home/lucks/anaconda3/bin/python

import rospy
from turtlesim.srv import Spawn, SpawnRequest, SpawnResponse
import numpy as np


if __name__ == '__main__':
    rospy.loginfo("This is turtle spawn [Python]")
    # 1.初始化节点
    rospy.init_node("spawn_node")
    # 2.申请客户端
    client = rospy.ServiceProxy("/spawn", Spawn)
    # 3.创建请求
    nameList = ["ZHANG3", "LI4", "WANG5", "ZHAO6"]
    add_turtle = SpawnRequest()

    for name in nameList:
        add_turtle.name = name
        add_turtle.x = np.random.randint(0, 20)
        add_turtle.y = np.random.randint(0, 20)
        client.wait_for_service()
        client.call(add_turtle)
        rospy.loginfo("Tutrle {} crated at [{},{}]".format(name, add_turtle.x, add_turtle.y))