#!/usr/bin/env python
import rospy
from gimbal_serializer.msg import status
from geometry_msgs.msg import Vector3Stamped

def gimbal_test():
    pub = rospy.Publisher('gimbal/control', Vector3Stamped, queue_size=1)
    rospy.init_node('gimbal_test_node', anonymous=True)
    rate = rospy.Rate(100)
    msg = Vector3Stamped()
    msg.header.frame_id = "test"
    msg.header.seq = 0
    msg.vector.x = 0.0
    msg.vector.y = 0.0
    msg.vector.z = 0.0

    while not rospy.is_shutdown():
        msg.header.seq = msg.header.seq + 1
        if msg.vector.z < 180.0:
            msg.vector.x = msg.vector.x + 0.0157
            msg.vector.y = msg.vector.y - 0.0157
            msg.vector.z = msg.vector.z + 1.0
        elif msg.vector.z >= 180.0:
            msg.vector.x = 0.0
            msg.vector.y = 0.0
            msg.vector.z = 0.0
        pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        gimbal_test()
    except rospy.ROSInterruptException:
        pass
