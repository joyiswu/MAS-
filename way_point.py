#!/usr/bin/env python

import sys
import rospy
import copy
import actionlib
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume, RobotTrajectory
from moveit_msgs.srv import GetPositionFK 
from std_msgs.msg import Header
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi
from math import cos, sin, tan
import numpy as np
import math

# Sampling the points in straight 
def straight_line_sample(start_point, end_point, resolution = 10):
    point_list = []
    point_list.append(start_point)

    # Calculate the distance between starting point and ending point 
    distance = np.sqrt(pow(start_point[0] - end_point[0], 2) + pow(start_point[1] - end_point[1], 2))

    # Calculate the number of the waypoints
    num_point = int(distance * resolution)

    if num_point > 1:

        # Add waypoints
        for i in range(1, num_point):
            x = start_point[0] + i * ((end_point[0] - start_point[0]) / (distance * resolution))
            y = start_point[1] + i * ((end_point[1] - start_point[1]) / (distance * resolution))
            point_list.append((x, y))

    point_list.append(end_point)

    return point_list

def two_point_print(goal):

    current_pose = group.get_current_pose()
    current_position = current_pose.pose.position

    # Calculate the current differential
    differential = get_differential((current_position.x, current_position.y), goal)

    # Get current joint value
    current_joint_value = group.get_current_joint_values()

    print(current_joint_value)

    if differential == 'inf': 
        current_joint_value[2] = 0
    else:
        current_joint_value[2] = math.atan(differential)

    group.go(current_joint_value)

    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x = goal[0]
    wpose.position.y = goal[1]
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    group.execute(plan, wait = True)

def get_differential(start_point, end_point):
    if (end_point[0] - start_point[0]) == 0: 
        return 'inf'
    else:
        return (end_point[1] - start_point[1]) / (end_point[0] - start_point[0])

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

    global group

    # Robot contains the entire state of the robot (iiw a and shadow hand)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('robot')
    arm_group = moveit_commander.MoveGroupCommander('arm')
    gripeer_group = moveit_commander.MoveGroupCommander('gripper')

    # Create a DisplayTrajectory publisher which is used later to publish trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

    # Set the number of planning attempts to find better solution 
    group.set_num_planning_attempts(10)

    """Get basic information"""
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()



################################################
###Add obstacle
    # rospy.sleep(2)
    # box_pose = geometry_msgs.msg.PoseStamped()
    # box_pose.header.frame_id = "odom"
    # box_pose.pose.position.x = 0
    # box_pose.pose.position.y = 3
    # box_pose.pose.position.z = 0.01
    # box_name = "wall"
    # scene.add_box(box_name, box_pose, size=(2, 2, 0.01))
    # rospy.sleep(0.5)
################################################


    a = straight_line_sample((-1,2), (-1,4))
    b = straight_line_sample((-1,4), (1,4))
    
    a = a + b

    pose = group.get_current_joint_values()

    constraints = Constraints()
    joint_constraint = JointConstraint()

    constraints.joint_constraints.append(JointConstraint(joint_name='arm_joint_1',
                                                         position = 0, tolerance_above=0,
                                                         tolerance_below=0, weight=1))

    constraints.joint_constraints.append(JointConstraint(joint_name='arm_joint_2',
                                                         position = pose[4], tolerance_above=0,
                                                         tolerance_below=0, weight=1))

    constraints.joint_constraints.append(JointConstraint(joint_name='arm_joint_3',
                                                         position = pose[5], tolerance_above=0,
                                                         tolerance_below=0, weight=1))

    constraints.joint_constraints.append(JointConstraint(joint_name='arm_joint_4',
                                                         position = pose[6], tolerance_above=0,
                                                         tolerance_below=0, weight=1))

    constraints.joint_constraints.append(JointConstraint(joint_name='arm_joint_5',
                                                         position = pose[7], tolerance_above=0,
                                                         tolerance_below=0, weight=1))
    group.set_path_constraints(constraints)

    pose = group.get_current_pose(group.get_end_effector_link())

    (roll, pitch, yaw) = euler_from_quaternion([pose.pose.orientation.x,
                                      pose.pose.orientation.y,
                                      pose.pose.orientation.z,
                                      pose.pose.orientation.w])

    differential = map(get_differential, a[:-1], a[1:])
    print(differential)
    for point in a:
        two_point_print(point)


    print 'all finish'

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        cw3_example_script()
    except rospy.ROSInterruptException:
        pass