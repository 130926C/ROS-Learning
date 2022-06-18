import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf

if __name__ == '__main__':
    # 1.初始化节点
    rospy.init_node("pub_py")
    # 2.创建发布对象
    pub = tf2_ros.StaticTransformBroadcaster()
    # 3.组织发布数据
    ts = TransformStamped()
    ts.header.frame_id = "base_line"
    ts.header.stamp = rospy.Time.now()
    ts.child_frame_id = "laser"
    #   设置相对位移
    ts.transform.translation.x = 2.0
    ts.transform.translation.y = 0.1
    ts.transform.translation.z = 3.2
    #   设置相对旋转
    qtn = tf.transformations.quaternion_from_euler(0,0,0)
    ts.transform.rotation.x = qtn[0]
    ts.transform.rotation.y = qtn[1]
    ts.transform.rotation.z = qtn[2]
    ts.transform.rotation.w = qtn[3]

    pub.sendTransform(ts)
    print("Messgae sended")
    rospy.spin()

    pass