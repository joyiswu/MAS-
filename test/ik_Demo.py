#!/usr/bin/env python

import sys
import rospy
import copy
import actionlib
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume, RobotTrajectory, PositionIKRequest
from moveit_msgs.srv import GetPositionIK, GetPositionFK
import geometry_msgs.msg
# from moveit_commander.conversions import pose_to_list
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, atan2
from math import cos, sin
from sensor_msgs.msg import JointState
import numpy as np

def get_ik(target):
    """
    :param target:  a PoseStamped give the desired position of the endeffector.
    """    

    pose = group.get_current_pose(group.get_end_effector_link())
    constraints = Constraints()
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header = pose.header
    orientation_constraint.link_name = group.get_end_effector_link()
    orientation_constraint.orientation = pose.pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.1
    orientation_constraint.absolute_y_axis_tolerance = 0.1
    orientation_constraint.absolute_z_axis_tolerance = 2*pi
    current_orientation_list = [pose.pose.orientation.x,
                                pose.pose.orientation.y,
                                pose.pose.orientation.z,
                                pose.pose.orientation.w]

    # get euler angle from quaternion
    (roll, pitch, yaw) = euler_from_quaternion(current_orientation_list)
    pitch = pi
    roll = 0
    orientation_constraint.weight = 1

    [orientation_constraint.orientation.x, orientation_constraint.orientation.y, orientation_constraint.orientation.z, orientation_constraint.orientation.w] = \
        quaternion_from_euler(roll, pitch, yaw)

    constraints.orientation_constraints.append(orientation_constraint) 
    # group.set_path_constraints(constraints) 
  #####################################################################  
    rospy.wait_for_service('compute_ik')
    request_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    service_request = PositionIKRequest()
    service_request.group_name = 'robot'
    service_request.pose_stamped.header.frame_id = 'base_footprint'
    # service_request.pose_stamped = group.get_current_pose()

    service_request.robot_state = robot.get_current_state()
    service_request.ik_link_name = 'arm_link_5'
    # Set position
    service_request.pose_stamped.pose.position.x = target[0]
    service_request.pose_stamped.pose.position.y = target[1]
    service_request.pose_stamped.pose.position.z = target[2]

    service_request.pose_stamped.pose.orientation.w =1

    service_request.constraints.orientation_constraints.append(orientation_constraint)
    service_request.timeout.secs= 4
    service_request.attempts= 2
    service_request.avoid_collisions = True

    resp = request_ik(service_request)
    return resp

def cw3_example_script():

    """
    This script will go through the main aspects of moveit and the components you will need to complete the coursework.
    You can find more information on
    """

    # Initialize moveit_commander and rospy node 
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface')

    # Initialize moveit scene interface (woeld surrounding the robot)
    scene = moveit_commander.PlanningSceneInterface()

    # Robot contains the entire state of the robot (iiw a and shadow hand)
    global group, base_group, robot

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('robot')
    arm_group = moveit_commander.MoveGroupCommander('arm')
    gripeer_group = moveit_commander.MoveGroupCommander('gripper')
    base_group = moveit_commander.MoveGroupCommander('base')

    # Create a DisplayTrajectory publisher which is used later to publish trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

    group.allow_replanning(True)
    group.allow_looking(True)
    # Set the number of planning attempts to find better solution 
    group.set_num_planning_attempts(10)

###Add obstacle
    rospy.sleep(0.5)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "odom"
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 3
    box_pose.pose.position.z = 0.01
    box_name = "wall"
    scene.add_box(box_name, box_pose, size=(2, 0.01, 0.01))
    rospy.sleep(0.5)

    rospy.sleep(0.5)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "odom"
    box_pose.pose.position.x = -0.8
    box_pose.pose.position.y = 3.5
    box_pose.pose.position.z = 0.2
    box_one = "box_1"
    scene.add_box(box_one, box_pose, size=(0.4, 0.4, 0.4))
    rospy.sleep(0.5)

    rospy.sleep(0.5)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "odom"
    box_pose.pose.position.x = -0
    box_pose.pose.position.y = 2.5
    box_pose.pose.position.z = 0.2
    box_two = "box_2"
    scene.add_box(box_two, box_pose, size=(0.4, 0.4, 0.4))
    rospy.sleep(0.5)

    rospy.sleep(0.5)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "odom"
    box_pose.pose.position.x = 0.8
    box_pose.pose.position.y = 3.5
    box_pose.pose.position.z = 0.2
    box_three = "box_3"
    scene.add_box(box_three, box_pose, size=(0.4, 0.4, 0.4))
    rospy.sleep(0.5)

#############################################################

#############################################################
    resp = get_ik([2.6, 3, 0.3])

    print len(resp.solution.joint_state.position) 

    # while(1):
    #     resp = get_ik([-1, 3, 2])

    #     if len(resp.solution.joint_state.position) != 0: break

    # print len(resp.solution.joint_state.position), resp.solution.joint_state.name




#############################################################

    print 'all finish'

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    rospy.init_node('move_group_python_interface')
    try:
        cw3_example_script()
    except rospy.ROSInterruptException:
        pass