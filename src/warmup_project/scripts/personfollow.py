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

## Simple person follow Neato deamo
## Neato turns to and moves towards the nearest object

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

    #Get data for all 360 degrees
    distance_measurements = []
    angles = range(360)
    for i in angles:
        #Only add "good measurements" to the list
        if msg.ranges[i] != 0 and msg.ranges[i] < 7 and msg.ranges[i] > 0.2:
            distance_measurements.append(msg.ranges[i])
        else:
            distance_measurements.append(1000)

def follow():
    """ Run loop for the wall node """
    global distance_measurements

    #Basic setup for the Neato
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, scan_received)
    rospy.init_node('wall', anonymous=True)

    #Variables for this function
    r = rospy.Rate(10) # 10hz
    desired_distance = 0.5
    state = 'idle'
    
    while not rospy.is_shutdown():

        #Only move the Neato if distance measurements have been taken so far
        if len(distance_measurements) != 0:
            #Get angle and distance from the minimum value in 360 degree measurements 
            angle = np.argmin(distance_measurements)
            minimum = distance_measurements[angle]

            #Set state based on if needs to turn right or left
            if angle > 180:
                state = "turn right"
            if angle < 180:
                state =  "turn left"

            #Set angle based on state
            if state == 'turn left':
                zOffset = angle * 0.01
            if state == 'turn right':
                zOffset = (360 - angle) * 0.01 * -1

            #Have Neato move porportially forwards to the nearest object
            xOffset = (minimum-desired_distance)*.2
            
            #Set message to Neato!
            msg = Twist(linear=Vector3(x=xOffset), angular=Vector3(z = zOffset))
            pub.publish(msg)
            r.sleep()
        
if __name__ == '__main__':
    try:
        follow()
    except rospy.ROSInterruptException: pass