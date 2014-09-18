#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan

distance_measurements = []

def scan_received(msg):
    """ Callback function for msg of type sensor_msgs/LaserScan """
    global distance_measurements

    distance_measurements = []
    angles = range(360)

    for i in angles:
        if msg.ranges[i] != 0 and msg.ranges[i] < 7 and msg.ranges[i] > 0.2:
            distance_measurements.append(msg.ranges[i])
        else:
            distance_measurements.append(1000)
    #print distance_measurements
    #print distance_measurements

def wall():
    global distance_measurements
    """ Run loop for the wall node """
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, scan_received)
    rospy.init_node('wall', anonymous=True)
    r = rospy.Rate(10) # 10hz

    desired_distance = 0.5
    
    while not rospy.is_shutdown():
        # print "distance_to_wall " + str(distance_to_wall)
        # print "right " + str(right)
        # print "left " + str(left)
        if len(distance_measurements) != 0:

            angle = np.argmin(distance_measurements)
            minimum = distance_measurements[angle]
            # print "angle: " + str(angle)
            # print "minimum: " + str(distance_measurements[angle])

            xOffset = (minimum-desired_distance)*.2

            if angle > 180:
                print "right"
                zOffset = (360 - angle) * 0.01 * -1

            if angle < 180:
                print "left"
                zOffset = angle * 0.01
            print zOffset

            
            msg = Twist(linear=Vector3(x=xOffset), angular=Vector3(z = zOffset))
            pub.publish(msg)
            r.sleep()
        
if __name__ == '__main__':
    try:
        wall()
    except rospy.ROSInterruptException: pass