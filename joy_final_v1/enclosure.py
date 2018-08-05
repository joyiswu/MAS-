#!/usr/bin/env python  
import sys
import rospy
import moveit_commander
import copy
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
from sensor_msgs.msg import JointState


class enclosure_demo:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('enclosure_node')

        scene = Scene_obstacle()
        self.control = arm_base_control()
        self.base_group = moveit_commander.MoveGroupCommander('base')

        # pose = control.base_group.get_current_pose(control.base_group.get_end_effector_link())
        # print(pose)


        check_inside = rospy.Subscriber("joint_states", JointState, self.check_inside_callback)

    def check_inside_callback(self,data):
        joint_data = data
        # self.base_group.set_position_target([0, 0, 0.05], self.base_group.get_end_effector_link())
        # result = self.base_group.plan()


        ## Replan to check for dynamic obstacle
        waypoints = []

        # Add the current pose to make sure the path is smooth, get latest pose
        current_pose = self.base_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(current_pose))

        # Add future printing obstacle
        (current_pose.position.x, current_pose.position.y, current_pose.position.z) = (0, 0, 0.05)
        waypoints.append(copy.deepcopy(current_pose))

        (plan2, fraction2) = self.base_group.compute_cartesian_path(waypoints, 0.01, 0.00)
        print fraction2



if __name__ == '__main__':
    try:
        enclosure_demo()
        while not rospy.is_shutdown():
            rospy.sleep(0.01)
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        pass