#!/usr/bin/env python

import sys
import rospy
import copy
import actionlib
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume, RobotTrajectory
import geometry_msgs.msg
# from moveit_commander.conversions import pose_to_list
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi
from math import cos
from sensor_msgs.msg import JointState

class joint_state_change:
    def __init__(self):
    # Initialize moveit_commander and rospy node 
        rospy.init_node('jointstate_change_node', anonymous=True)
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.joint_data = JointState()

        rospy.Subscriber("joy/joint_states", JointState, self.joint_callback)

        cw3_example_script()
        rospy.spin()

    def joint_callback(self, data):

        self.joint_data = data
        if self.joint_data.position[2] > 2*pi: elf.joint_data.position[2] = 2*pi
        if self.joint_data.position[2] < -2*pi: elf.joint_data.position[2] = -2*pi
        print(self.joint_data.position)
        self.pub.publish(self.joint_data)


if __name__ == '__main__':
    rospy.init_node('move_group_python_interface')
    try:
        joint_state_change()
    except rospy.ROSInterruptException:
        pass