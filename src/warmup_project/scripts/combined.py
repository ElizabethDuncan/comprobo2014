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

## Simple Neato demo that has the Neato follow a wall and person follow
## Switches to wallfollow when there is no object within 1 meters
## Switches to person follow when there is an object within .4 m in front of it
## CompRobo - 9/19

import rospy
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan

distance_to_forward_wall = -1.0
distance_to_backwards_wall = -1.0
distance_from_side = -1.0
desired_distance = 1
distance_from_front = 0
behavior = 'wallfollow'
distance_measurements = []

def scan_received(msg):
    """ Callback function for msg of type sensor_msgs/LaserScan """
    """ Collect data for for personfollow and wallfollow """

    #Variables used to collect data from scan node
    global desired_distance, distance_to_forward_wall, distance_to_backwards_wall, distance_from_side, distance_from_front, behavior, distance_measurements
 
    #If behavior is wallfollow, collect wallfollow data
    if behavior == 'wallfollow':
        valid_measurements = []
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
        for i in angle_ninety:
            if msg.ranges[i] != 0 and msg.ranges[i] < 7:
                valid_measurements.append(msg.ranges[i])
        if len(valid_measurements): 
            distance_from_side = sum(valid_measurements)/float(len(valid_measurements))
        else:
            distance_from_side = -1.0

        #Go through degrees near 0, average those and assign to front
        for i in angle_front:
            if msg.ranges[i] != 0 and msg.ranges[i] < 7:
                valid_measurements.append(msg.ranges[i])
        if len(valid_measurements): 
            distance_from_front = sum(valid_measurements)/float(len(valid_measurements))
        else:
            distance_from_front = -1

    #If behavior is personfolllow, collect personfollow data
    elif behavior == 'personfollow':
        #Get data for all 360 degrees
        distance_measurements = []
        angles = range(360)
        for i in angles:
            #Only add "good measurements" to the list
            if msg.ranges[i] != 0 and msg.ranges[i] < 7 and msg.ranges[i] > 0.2:
                distance_measurements.append(msg.ranges[i])
            else:
                distance_measurements.append(1000)



def wall():
    """ Function called for running of neato """
    """ Loops over wallfollow or personfollow behavior"""
    global desired_distance, distance_to_forward_wall, distance_to_backwards_wall, distance_from_side, distance_from_front, behavior, distance_measurements

   #Set up for communication with the cmd_vel and scan node - used by the Neato
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, scan_received)
    rospy.init_node('wall', anonymous=True)
    r = rospy.Rate(10) # 10hz

    #Set default state
    state = 'idle'
    
    
    while not rospy.is_shutdown():
        #Print current behavior - either wallfollow or person follow
        print "behavior: " + str(behavior)

        #If behavior is wallfollow, execute wallfollow behavior
        if behavior == 'wallfollow':
            #Stop Neato and change behaviors
            if distance_from_front > 0  and distance_from_front < 0.4:
                #Stop Neato briefly
                pub.publish(Twist())
                print "switching behavior!"
                behavior = 'personfollow'
                continue

            #Assign states for wallfollow
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

        #If behavior is personfollow, execute personfollow behavior
        elif behavior == 'personfollow':
            #Only move the Neato if distance measurements have been taken so far
            if len(distance_measurements) != 0:
                #Get angle and distance from the minimum value in 360 degree measurements 
                angle = np.argmin(distance_measurements)
                minimum = distance_measurements[angle]

                #Stop Neato and change states
                if minimum > 1:
                    print "Stopping because something is in front!"
                    pub.publish(Twist())
                    print "switching behavior to wallfollow"
                    behavior = 'wallfollow'
                    continue

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
        wall()
    except rospy.ROSInterruptException: pass