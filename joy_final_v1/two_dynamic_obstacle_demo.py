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
from control_command import arm_base_control

class dynamic_obstacle_demo:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('dynamic_obstacle_demo')

        scene = Scene_obstacle()
        control = arm_base_control()

        sce = moveit_python.PlanningSceneInterface('odom')
        sce.clear()

        # scene.scene.remove_world_object()
        rospy.sleep(1)

        point_list = scene.straight_line_sample((-1,1), (3,1))
        point_list_ob = scene.straight_line_sample((-1,1), (3,1), height = 0.05)
        scene.print_list_visualize(point_list_ob)
        control.print_pointlist(point_list)

        
        # # Happy ending
        # control.group.set_position_target([0, 0, 0.1], control.group.get_end_effector_link())

        # control.group.go(wait = False)



if __name__ == '__main__':
    try:
        dynamic_obstacle_demo()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        pass