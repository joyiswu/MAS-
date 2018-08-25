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
from obstacle_command import Scene_obstacle
from control_robot1 import arm_base_control
import re
import time
import datetime

class general_printing_demo:
    def __init__(self):

        # moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('general_printing')

        control = arm_base_control()

        point_list = []
        # point_list += control.get_circle_point((-1,1), 1)
        point_list += control.straight_line_sample((0,1), (1,1))
        point_list += control.straight_line_sample((1,1), (1,2))
        point_list += control.straight_line_sample((1,2), (3,1))
        point_list += control.straight_line_sample((3,1), (1,0))
        point_list += control.straight_line_sample((1,0), (1,1))

        # control.print_list_visualize(point_list)

        control.print_pointlist(point_list)

  


if __name__ == '__main__':
    try:
        general_printing_demo()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        pass