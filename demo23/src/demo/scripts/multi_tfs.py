import rospy
from tf2_geometry_msgs import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import TransformStamped


if __name__ == "__main__":
    # 1.初始化节点
    rospy.init_node("sub_node")
    # 2.创建一个son1坐标系下的点
    psInson1 = tf2_geometry_msgs.PointStamped()
    psInson1.header.frame_id = "son1"
    psInson1.header.stamp = rospy.Time.now()
    
    psInson1.point.x = 1.0
    psInson1.point.y = 2.0
    psInson1.point.z = 3.0

    # 3.创建一个订阅对象
    buffer = tf2_ros.buffer
    listener = tf2_ros.TransformListener(buffer)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        try:
            # 1.将son1和son2的相对坐标关系转化
            ts = buffer.lookup_transform("son2", "son1", rospy.Time(0))
            rospy.loginfo("Father:%s, Son:%s, bias value=(%.2f, %.2f, %.2f)", 
                ts.header.frame_id,
                ts.child_frame_id,
                ts.transform.translation.x,
                ts.transform.translation.y,
                ts.transform.translation.z
            )

            psInson2 = buffer.transform(psInson1, "son2")
            rospy.loginfo("after trans point (%.2f, %.2f, %.2f), relative system:%s",
                psInson2.point.x,
                psInson2.point.y,
                psInson2.point.z,
                psInson2.header.frame_id
            )
        except Exception as e:
            rospy.loginfo("Error Msgs: %s", e)