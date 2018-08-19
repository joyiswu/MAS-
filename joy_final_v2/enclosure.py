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
from control_test import arm_base_control

class enclosure_demo:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('enclosure_demo')

        control = arm_base_control('robot0')

        # sce = moveit_python.PlanningSceneInterface('odom')
        # sce.clear()

        rospy.sleep(0.5)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "odom"
        box_pose.pose.position.x = -0.5
        box_pose.pose.position.y = 2.5
        box_pose.pose.position.z = 0.2
        box_one = "box_1"
        control.scene.add_box(box_one, box_pose, size=(0.5, 0.5, 0.5))
        rospy.sleep(0.5)

        # scene.scene.remove_world_object()
        rospy.sleep(1)

        point_list = []
        point_list += control.straight_line_sample((-1,1), (2,1))
        point_list += control.straight_line_sample((2,1), (2,3))
        point_list += control.straight_line_sample((1,3), (0,3))
        point_list += control.straight_line_sample((0,3), (0,1))



        # point_list_ob = scene.get_circle_point((2,2), 1, height = 0.05)
        # control.print_list_visualize(point_list)
        # control.print_list_visualize(point_list_0, name = 'future_ob')
        control.print_pointlist(point_list, future_print_status = False)

        
        # # Happy ending
        # control.group.set_position_target([0, 0, 0.1], control.group.get_end_effector_link())

        # control.group.go(wait = False)



if __name__ == '__main__':
    try:
        enclosure_demo()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        pass