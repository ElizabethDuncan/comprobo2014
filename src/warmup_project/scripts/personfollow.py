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

## Simple Neato demo that follows a person (or mass) within its field of vision

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan


class PersonFollower():

    def __init__(self):
        #Set up for communication with the cmd_vel and scan node - used by the Neato
        self.valid_measurements = []
        self.front_measurements = []
        self.last_measurement = []
        self.first_run = True
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_received)
        rospy.init_node('wall', anonymous=True)
        self.r = rospy.Rate(10) # 10hz

    def scan_received(self, msg):
        """ Callback function for msg of type sensor_msgs/LaserScan """
        self.valid_measurements = []
        angle_front = [315, 316, 317, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327, 328, 329, 330, 331, 332, 333, 334, 335, 336, 337, 339, 340, 341, 342, 343, 344, 345, 346, 347, 348, 349, 350, 351, 352, 353, 354, 355, 356, 357, 358, 359, 1, 2, 3, 4,5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45]
        forward_measurements = [355, 356, 357, 358, 359, 1, 2, 3, 4, 5]
        #Go through degrees near 0, average those and assign to front
        for i in angle_front:
            if  msg.ranges[i] > 7:
                self.valid_measurements.append(0.0)
            else:
                self.valid_measurements.append(round(msg.ranges[i], 1))

        #Go through degrees near 0, average those and assign to front
        for i in forward_measurements:
            if msg.ranges[i] != 0 and msg.ranges[i] < 7:
                self.front_measurements.append(msg.ranges[i])
        if len(self.front_measurements): 
            self.distance_from_front = sum(self.front_measurements)/float(len(self.front_measurements))
        else:
            self.distance_from_front = -1

    def wall(self):
        zOffset = 0
        xOffset = 0
        print self.first_run
        while not rospy.is_shutdown():
            #print self.first_run
            #Set default state
            state = 'idle'
            #print self.valid_measurements
            for index in range(0, len(self.valid_measurements)):
                #print "Compare: " + str(index)
                
                if self.first_run == False:
                    if self.valid_measurements[index] != self.last_measurement[index] and self.valid_measurements[index] != 0.0 and self.last_measurement[index] != 0.0:
                        difference = abs(self.valid_measurements[index] - self.last_measurement[index])
                        if difference > 0.3:
                            print "difference!"
                            print index
                            if index < 45:
                                zOffset = ((45 - index) * 0.01) *-1
                            if index > 45:
                                zOffset = ((45 - index) * 0.01) *-1
                    if self.distance_from_front != 1:
                        xOffset = self.distance_from_front-1
                        print xOffset
                    else:
                        xOffset = 0

            #Show Neato state and send appropriate message to Neato
            #print state
            msg = Twist(linear=Vector3(x=xOffset), angular=Vector3(z = zOffset))
            self.pub.publish(msg)
            self.last_measurement = self.valid_measurements
            self.first_run = False
            self.r.sleep()
        
if __name__ == '__main__':
    try:
        person_follower = PersonFollower()
        person_follower.wall()

    except rospy.ROSInterruptException: pass