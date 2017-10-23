#!/usr/bin/env python
# This script should return the x y z and orientation coordinates of the end effector of the left limb.
# PLEASE ADD YOUR CODE WHERE INDICATED
# Avoid modifying the rest of the code if not necessary
#
# Authors: Stefano Pietrosanti - s.pietrosanti@pgr.reading.ac.uk
#          Guy Butcher

import rospy
import baxter_interface
import numpy
import math
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

def getNormalVector2():
    rospy.init_node("SSE_forward_kinematics")
    left_arm = baxter_interface.Limb("left")
    print 'move baxter to point one then press enter to record'
    raw_input()
    pose = left_arm.endpoint_pose();
    print pose['orientation']
    p1 = [pose['position'].x, pose['position'].y, pose['position'].z]
    print 'point 1: ', p1
    print 'move baxter to point two then press enter to record'
    raw_input()
    pose = left_arm.endpoint_pose();
    p2 = [pose['position'].x, pose['position'].y, pose['position'].z]
    print 'point 2: ', p2
    print 'move baxter to point three then press enter to record'
    raw_input()    
    pose = left_arm.endpoint_pose();
    p3 = [pose['position'].x, pose['position'].y, pose['position'].z]
    print 'point 3: ', p3
    print 'beginning calculation of normal vector'
    p12 = [p1[0] - p2[0], p1[1] - p2[1], p1[2]-p2[2]]
    p13 = [p1[0] - p3[0], p1[1] - p3[1], p1[2]-p3[2]]
    print 'vector 1: ', p12
    print 'vector 2: ', p13

    crossVector = numpy.cross(p12, p13)
  
    print 'cross vector', crossVector
    crossVector_norm = numpy.zeros(3)
    magnitude = (math.sqrt(crossVector[0]**2) + math.sqrt(crossVector[1]**2) + math.sqrt(crossVector[2]**2) )
    crossVector_norm[0] = crossVector[0] / magnitude
    crossVector_norm[1] = crossVector[1] / magnitude
    crossVector_norm[2] = crossVector[2] / magnitude
    print 'normalized cross vector', crossVector_norm
    

getNormalVector2()

print("MIM tutorial: forward kinematics.")
# Initialising ROS node
rospy.init_node("SSE_forward_kinematics")

######################  INSERT YOUR CODE HERE
# Create a "Limb" instance called "left_arm" linked to Baxter's left limb
left_arm = baxter_interface.Limb("left")
	

# Create a "pose" variable which holds the output of endpoint_pose()
pose = left_arm.endpoint_pose();
angles = left_arm.joint_angles();
print angles

lcmd={'left_w0':-0.381669966846, 'left_w1': 0.22169442983, 'left_w2': -0.98, 'left_e0': -0.751267090009, 'left_e1': 1.02009722278, 'left_s0': 0.335558296967, 'left_s1': -0.214757310059}
print lcmd['left_w0']
left_arm.move_to_joint_positions(lcmd)
left_arm.move_to_joint_positions(angles)

print lcmd
######################



# Return pose
#print("Endpoint coordinates:")
#print("X: " + str(pose['position'].x))
#print("Y: " + str(pose['position'].y))
#print("Z: " + str(pose['position'].z))

