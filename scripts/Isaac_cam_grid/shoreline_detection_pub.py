#!/usr/bin/env python
#publish position for moving camera over the map to create images for shoreline localization, flying over the map in a grid
# y: [-1000, -400] and x: [-1800, 650], and z: 100.

import rospy
from geometry_msgs.msg import Pose

def talker():
    pub = rospy.Publisher('/sl_detection_pose', Pose, queue_size=10)
    rospy.init_node('sl_detection_pub', anonymous=True)
    rate = rospy.Rate(0.25) # 0.25 Hz
    sl_detection_pose = Pose()
    sl_detection_pose.position.y = -1000
    sl_detection_pose.position.x = 650
    sl_detection_pose.position.z = 100
    sl_detection_pose.orientation.w = 0.70711
    sl_detection_pose.orientation.x = 0.70711
    sl_detection_pose.orientation.y = 0
    sl_detection_pose.orientation.z = 0

    while not rospy.is_shutdown():
        if sl_detection_pose.position.x == -1800:
            sl_detection_pose.position.x = 650
            sl_detection_pose.position.y += 50
        sl_detection_pose.position.x -= 50
        pub.publish(sl_detection_pose)
        print(sl_detection_pose)
        rate.sleep()     # wait for 4 sec


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
