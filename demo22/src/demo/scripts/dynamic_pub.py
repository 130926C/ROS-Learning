import rospy
from turtlesim.msg import Pose
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf_conversions

def doPose(pose):
    # 1.创建一个坐标系相对关系的对象
    pub = tf2_ros.TransformBroadcaster()
    # 2.转换坐标信息
    ts = TransformStamped()
    ts.header.frame_id = "world"
    ts.header.stamp = rospy.Time.now()
    ts.child_frame_id = "turtle1"

    ts.transform.translation.x = pose.x
    ts.transform.translation.y = pose.y
    ts.transform.translation.z = 0

    qtn = tf_conversions.transformations.quaternion_from_euler(0, 0, pose.theta)
    ts.transform.rotation.x = qtn[0]
    ts.transform.rotation.y = qtn[1]
    ts.transform.rotation.z = qtn[2]
    ts.transform.rotation.w = qtn[3]

    # 3.发布
    pub.sendTransform(ts)

if __name__ == "__main__":
    # 1.初始化节点
    rospy.init_node("pub_node")
    # 2.订阅乌龟的位姿信息
    sub = rospy.Subscriber("/turtle1/pose", Pose, doPose, queue_size=100)
    # 3.将位姿信息转化为相对坐标系 callback function

    rospy.spin()

