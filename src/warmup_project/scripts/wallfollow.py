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

def scan_received(msg):
    """ Callback function for msg of type sensor_msgs/LaserScan """
    global desired_distance
    global distance_to_forward_wall
    global distance_to_backwards_wall
    global distance_from_side
    valid_measurements = []
    angles_to_check = [355, 356, 357, 358, 359, 0, 1, 2, 3, 4]
    angle_fourtyfive = [40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50]
    angle_onethirtyfive = [130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140]
    angle_ninety = [85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95]
    #Go through degrees near 45, average those and assing to distance_to_forward_wall
    for i in angle_fourtyfive:
        if msg.ranges[i] != 0 and msg.ranges[i] < 7:
            valid_measurements.append(msg.ranges[i])
    if len(valid_measurements):
        distance_to_forward_wall = sum(valid_measurements)/float(len(valid_measurements))
    else:
        distance_to_forward_wall = -1.0
    #print "distance_to_forward_wall: " + str(distance_to_forward_wall)

    #Go through degrees near 135, average those and assing to distance_to_backward_wall
    for i in angle_onethirtyfive:
        if msg.ranges[i] != 0 and msg.ranges[i] < 7:
            valid_measurements.append(msg.ranges[i])
    if len(valid_measurements): 
        distance_to_backwards_wall = sum(valid_measurements)/float(len(valid_measurements))
    else:
        distance_to_backwards_wall = -1.0
    #print "distance_to_backwards_wall: " + str(distance_to_backwards_wall)

    #Go through degrees near 135, average those and assing to distance_to_backward_wall
    for i in angle_ninety:
        if msg.ranges[i] != 0 and msg.ranges[i] < 7:
            valid_measurements.append(msg.ranges[i])
    if len(valid_measurements): 
        distance_from_side = sum(valid_measurements)/float(len(valid_measurements))
    else:
        distance_from_side = -1.0
    #print "distance_from_side: " + str(distance_from_side)

def getch():
    """ Return the next character typed on the keyboard """
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def wall():
    global desired_distance
    global distance_to_forward_wall
    global distance_to_backwards_wall
    global distance_from_side
    """ Run loop for the wall node """
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, scan_received)
    rospy.init_node('wall', anonymous=True)
    r = rospy.Rate(10) # 10hz
    forwardOffset = desired_distance - distance_to_forward_wall 
    #forwardOffset = -1 * forwardOffset
    
    backwardsOffset = desired_distance - distance_to_backwards_wall
    while not rospy.is_shutdown():
        if distance_to_backwards_wall == -1 or distance_to_forward_wall == -1:
            print "bad valid_measurements"
            if distance_from_side != -1 and distance_from_side < 1:
                zOffset = 0.05
                forward = 0.05
            elif distance_from_side > 1: 
                zOffset = -0.05
                forward = 0.05
            else:
                print "NO MEASUREMENTS"
                zOffset = 0
                forward = 0
        elif distance_to_forward_wall > distance_to_backwards_wall:
            zOffset = (desired_distance - distance_to_backwards_wall)*1
            print "forward bigger: " + str(zOffset)
            if zOffset > 0.1:
                zOffset = 0.1
            forward = 0.1
        else:
            zOffset = (desired_distance - distance_to_forward_wall)*1
            print "forward bigger: " + str(zOffset)
            forward = 0.1
        # if distance_from_side < desired_distance:
        #     print "move away"
        #     zOffset = 0.2
        # else:
        #     print "move closer"
        #     zOffset = -0.2
        # if distance_to_forward_wall == desired_distance and distance_to_backwards_wall == desired_distance:
        #     print "offset " + str(forwardOffset)
        #     msg = Twist()
        # else:
        #IMPLEMENT CORRECT MESSAGE
        msg = Twist(linear=Vector3(x=forward), angular=Vector3(z = (-zOffset)))
        #msg = Twist()
        pub.publish(msg)
        r.sleep()
        
if __name__ == '__main__':
    try:
        wall()
    except rospy.ROSInterruptException: pass