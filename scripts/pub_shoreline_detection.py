#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

def talker():
    pub = rospy.Publisher('/sl_detection_pose', Pose, queue_size=10)
    rospy.init_node('sl_detection_pub', anonymous=True)
    rate = rospy.Rate(0.5) # 0.5 Hz
    sl_detection_pose = Pose()
    sl_detection_pose.position.y = -350
    sl_detection_pose.position.x = 750
    sl_detection_pose.position.z = 20
    sl_detection_pose.orientation.w = 0.70711
    sl_detection_pose.orientation.x = 0.70711
    sl_detection_pose.orientation.y = 0
    sl_detection_pose.orientation.z = 0

    while not rospy.is_shutdown():        
        sl_detection_pose.position.x -= 15
        pub.publish(sl_detection_pose)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass