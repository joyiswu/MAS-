#!/usr/bin/env python  
import sys
import rospy
import moveit_commander
from cw3_helper.srv import ChangeCollisionObject
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi
from math import cos, sin
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
import geometry_msgs.msg
from geometry_msgs.msg import * 
import moveit_python



# There two obstacle move between (-1,1) and (3, 1)
if __name__ == '__main__':
    rospy.init_node('dynamic_obstacle')

    # scene = moveit_python.PlanningSceneInterface()

    scene = moveit_python.PlanningSceneInterface('odom')
    scene.clear()

    box1_x = -1
    box1_y = 1.5
    box1_z = 0.25

    box2_x = 3
    box2_y = 0.5
    box2_z = 0.25

    direction_1 = 1
    direction_2 = -1

    while not rospy.is_shutdown():

        if box1_x >= 3:  direction_1 = -1
        if box1_x <= -1: direction_1 = 1

        if box2_x >= 3:  direction_2 = -1
        if box2_x <= -1: direction_2 = 1
        box1_x = box1_x + 0.006 * direction_1
        box2_x = box2_x + 0.006 * direction_2

        scene.addBox('box_1',0.5,0.5,0.5, box1_x, box1_y,box1_z)
        scene.addBox('box_2',0.5,0.5,0.5, box2_x ,box2_y,box2_z)
        rospy.sleep(0.2)
