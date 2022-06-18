from http import client
import rospy
from turtlesim.srv import SpawnRequest, Spawn


if __name__ == "__main__":
    # 1.启动节点
    rospy.init_node("turtle2")
    # 2.获得一个发布者
    pub = rospy.ServiceProxy("/spawn", Spawn)

    # 3.准备一个发布消息
    spawn = SpawnRequest()
    spawn.name = "turtle2"
    spawn.x = 1.0
    spawn.y = 2.0
    spawn.theta = 0.0
    
    pub.wait_for_service()
    try:
        response = pub.call(spawn)
        rospy.loginfo("Spawn successed! turtlename is:%s", response.name)
    except Exception as e:
        rospy.logerr("Error, Msgs")    