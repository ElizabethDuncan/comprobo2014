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
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan

distance_to_forward_wall = -1.0
distance_to_backwards_wall = -1.0
distance_from_side = -1.0
desired_distance = 1
distance_from_front = 0

def scan_received(msg):
    """ Callback function for msg of type sensor_msgs/LaserScan """

    #Variables used to collect data from scan node
    global desired_distance
    global distance_to_forward_wall
    global distance_to_backwards_wall
    global distance_from_side
    global distance_from_front
    valid_measurements = []
    angles_to_check = [355, 356, 357, 358, 359, 0, 1, 2, 3, 4]
    angle_fourtyfive = [40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50]
    angle_onethirtyfive = [130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140]
    angle_ninety = [85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95]
    angle_front = [355, 356, 357, 358, 359, 1, 2, 3, 4, 5]

    #Go through degrees near 45, average those and assign to distance_to_forward_wall
    for i in angle_fourtyfive:
        if msg.ranges[i] != 0 and msg.ranges[i] < 7:
            valid_measurements.append(msg.ranges[i])
    if len(valid_measurements):
        distance_to_forward_wall = sum(valid_measurements)/float(len(valid_measurements))
    else:
        distance_to_forward_wall = -1.0

    #Go through degrees near 135, average those and assign to distance_to_backward_wall
    for i in angle_onethirtyfive:
        if msg.ranges[i] != 0 and msg.ranges[i] < 7:
            valid_measurements.append(msg.ranges[i])
    if len(valid_measurements): 
        distance_to_backwards_wall = sum(valid_measurements)/float(len(valid_measurements))
    else:
        distance_to_backwards_wall = -1.0

    #Go through degrees near 135, average those and assign to distance_to_backward_wall
    for i in angle_front:
        if msg.ranges[i] != 0 and msg.ranges[i] < 7:
            valid_measurements.append(msg.ranges[i])
    if len(valid_measurements): 
        distance_from_side = sum(valid_measurements)/float(len(valid_measurements))
    else:
        distance_from_side = -1.0

    #Go through degrees near 0, average those and assign to front
    for i in angle_ninety:
        if msg.ranges[i] != 0 and msg.ranges[i] < 7:
            valid_measurements.append(msg.ranges[i])
    if len(valid_measurements): 
        distance_from_front = sum(valid_measurements)/float(len(valid_measurements))
    else:
        distance_from_front = -1

def wall():
    global desired_distance
    global distance_to_forward_wall
    global distance_to_backwards_wall
    global distance_from_side
    global distance_from_front
   
   #Set up for communication with the cmd_vel and scan node - used by the Neato
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, scan_received)
    rospy.init_node('wall', anonymous=True)
    r = rospy.Rate(10) # 10hz

    #Set default state
    state = 'idle'
    
    #Assign states
    while not rospy.is_shutdown():
        #Check if needs to stop program
        if distance_from_front > 0  and distance_from_front < 0.4:
            print "Stopping because something is in front!"
            pub.publish(Twist())
            break

        if distance_to_backwards_wall == -1 or distance_to_forward_wall == -1:
            #No measurements coming in from 45 and 135 degrees - use 90 degrees measurments instead
            if distance_from_side != -1 and distance_from_side < 1:
                state = 'side too close'
            elif distance_from_side > 1: 
                state = 'side too far'
            else:
                state =  'NO MEASUREMENTS'
        elif distance_to_forward_wall > distance_to_backwards_wall:
            state = 'front too close'
            if zOffset > 0.1:
                state = 'front WAY too close'
        else:
            state = 'back too close'

        #Based on states, assign values to forward and angular motion
        if state == 'side too close':
            zOffset = 0.05
            forward = 0.05
        elif state == 'side too far':
            zOffset = -0.05
            forward = 0.05
        elif state == 'NO MEASUREMENTS':
            zOffset = 0
            forward = 0
        elif state == 'front too close':
            zOffset = (desired_distance - distance_to_backwards_wall)*1
            forward = 0.1
        elif state == 'front WAY too close':
            zOffset = 0.1
            forward = 0.1
        elif state == 'back too close':
            zOffset = (desired_distance - distance_to_forward_wall)*1
            forward = 0.1

        #Show Neato state and send appropriate message to Neato
        print state
        msg = Twist(linear=Vector3(x=forward), angular=Vector3(z = (-zOffset)))
        pub.publish(msg)
        r.sleep()
        
if __name__ == '__main__':
    try:
        wall()
    except rospy.ROSInterruptException: pass