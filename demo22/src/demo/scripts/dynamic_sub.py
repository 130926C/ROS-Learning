import rospy
import tf2_ros
from tf2_geometry_msgs import tf2_geometry_msgs

if __name__ == "__main__":
    # 1.初始化节点
    rospy.init_node("sub_node")
    # 2.创建订阅对象
    buffer = tf2_ros.Buffer()
    sub = tf2_ros.TransformListener(buffer)
    # 3.组织被转移的坐标点
    ps = tf2_geometry_msgs.PointStamped()
    ps.header.frame_id = "turtle1"
    ps.header.stamp = rospy.Time()
    ps.point.x = 1.0
    ps.point.y = 2.0
    ps.point.z = 3.0

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            ps_out = buffer.transform(ps, "world")
            rospy.loginfo("After trans pos:(%.2f, %.2f, %.2f), relative system:%s", 
                ps_out.point.x,
                ps_out.point.y,
                ps_out.point.z, 
                ps_out.header.frame_id
            )
        except Exception as e:
            rospy.logwarn("Error:%s", e)

        rate.sleep()
        rospy.spin()