#!/usr/bin/env python

#This program simulates the movement of a boat in moderate wave conditions
#Limited in pitch and roll to +- 10 deg (+-0.17 rad)

MAX_ROLLANGLE = 0.02
MAX_PITCHANGLE = 0.08

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState

def talker():
    global rollangle, pitchangle
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size = 10)
    rospy.init_node('water_move', anonymous=True)
    rate = rospy.Rate(50)
    uproll = 1
    uppitch = 1
    while not rospy.is_shutdown():        
        
        if (rollangle < MAX_ROLLANGLE / 2) & (uproll == 1):
            rollangle += 0.0001
        else:
            uproll = 0
            rollangle -= 0.0001
            if (rollangle < -MAX_ROLLANGLE / 2):
                uproll = 1

        if (pitchangle < MAX_PITCHANGLE / 2) & (uppitch == 1):
            pitchangle += 0.0003
        else:
            uppitch = 0
            pitchangle -= 0.0003
            if (pitchangle < -MAX_PITCHANGLE / 2):
                uppitch = 1        


        #print(rollangle)

        quat = get_quaternion_from_euler(rollangle, pitchangle, yawangle)
        shippose.orientation.x = quat[0]
        shippose.orientation.y = quat[1]
        shippose.orientation.z = quat[2]
        shippose.orientation.w = quat[3]
        shipstate.pose = shippose
        #shipstate.pose.position.x = 955
        #shipstate.pose.position.y = -882
        #shipstate.pose.position.z = 0

        for i in range(7):
            shipstate.pose.position.x = 955
            shipstate.pose.position.y = -882 - 10 * i
            shipstate.pose.position.z = 0
            shipstate.model_name = ship_names[i]
            pub.publish(shipstate)
       
        rate.sleep()

def get_quaternion_from_euler(roll, pitch, yaw):
     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
     return [qx, qy, qz, qw]



ship_names = ['vessel_a', 'vessel_b', 'vessel_c', 'vessel_d', 'vessel_e', 'vessel_f', 'vessel_g']
rollangle = 0.00
pitchangle = 0.05
yawangle = 0.00


shippose = Pose()
shipstate = ModelState()

quat = []

try:
    talker()
except rospy.ROSInterruptException:
    pass