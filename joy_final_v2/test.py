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
import re
import copy


class dynamic_obstacle_demo:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('dynamic_obstacle_demo')

        control = arm_base_control()
        # Check for enclosure
        control.base_group.set_position_target([0, 0, 0.05], control.base_group.get_end_effector_link())
        result = control.base_group.plan()
        control.base_group.clear_pose_targets()
        
        if len(result.joint_trajectory.points) == 0: 
            print('Check enclosure failed')
        else: print('Check enclosure successful')

        waypoints = []

        # Add the current pose to make sure the path is smooth, get latest pose
        current_ee_pose_smooth = control.base_group.get_current_pose().pose
        
        (current_ee_pose_smooth.position.x, current_ee_pose_smooth.position.y, current_ee_pose_smooth.position.z) = (0, 0, 0.05)
        waypoints.append(copy.deepcopy(current_ee_pose_smooth))

        (current_ee_pose_smooth.position.x, current_ee_pose_smooth.position.y, current_ee_pose_smooth.position.z) = (0, 0, 0.05)
        waypoints.append(copy.deepcopy(current_ee_pose_smooth))

        (plan2, fraction2) = control.base_group.compute_cartesian_path(waypoints, 0.01, 0.00)
        print fraction2

        # sce = moveit_python.PlanningSceneInterface('odom')
    
        # current_future_ob_list = sce.getKnownCollisionObjects()

        # for point_name in current_future_ob_list:
        #     result = re.match('printing', point_name)
        #     if result: 
        #         current_future_ob_list.append(point_name)
        #         sce.removeCollisionObject(point_name)


        # for name in current_future_ob_list:
        #     sce.removeCollisionObject(name)

 


if __name__ == '__main__':
    try:
        dynamic_obstacle_demo()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        pass