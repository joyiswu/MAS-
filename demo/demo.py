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
from math import pi, atan2
from math import cos, sin
from sensor_msgs.msg import JointState
import numpy as np

def rotation_circle(circle_center, radius, theta):

    if theta > pi: theta -= 2 * pi
    if theta < pi: theta += 2 * pi   

    x = circle_center[0] + radius * cos(theta)
    y = circle_center[1] + radius * sin(theta)

    return (x, y)

def rotation(angle, resolution = 2):
    position_eef = group.get_current_pose().pose.position

    position_base = base_group.get_current_pose().pose.position

    joint_value = group.get_current_joint_values()
    current_theta = atan2(position_base.y - position_eef.y, position_base.x - position_eef.x)

    # Calculate the distance between starting point and ending point 
    radius = np.sqrt(pow(position_eef.x - position_base.x, 2) + pow(position_eef.y - position_base.y, 2))

    # Calculate
    angle_c = np.arcsin(0.024 * sin(pi - joint_value[3] + 2.9496) / radius)
    angle_difference = joint_value[3] - 2.9496 - angle_c

    waypoints = []

    wpose = base_group.get_current_pose().pose
    print( base_group.get_current_pose().pose)
    (roll, pitch, yaw) = euler_from_quaternion([wpose.orientation.x,
                                wpose.orientation.y,
                                wpose.orientation.z,
                                wpose.orientation.w])

    different = current_theta  + angle_difference - (yaw-pi)
    # waypoints.append(copy.deepcopy(wpose))


    for i in range(0, angle + resolution, resolution):
        (x, y) = rotation_circle((position_eef.x, position_eef.y), radius, pi/180*i + current_theta)

        wpose.position.x = x
        wpose.position.y = y
        (roll, pitch, yaw) = euler_from_quaternion([wpose.orientation.x,
                                                    wpose.orientation.y,
                                                    wpose.orientation.z,
                                                    wpose.orientation.w])
        yaw =  pi/180*i + current_theta  + angle_difference - different

        [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w] = \
            quaternion_from_euler(roll, pitch, yaw - pi)

        # Add way point
        waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = base_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    base_group.execute(plan)

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
    global group, base_group

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
################################################
###Add obstacle
    rospy.sleep(2)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "odom"
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 3
    box_pose.pose.position.z = 0.01
    box_name = "wall"
    scene.add_box(box_name, box_pose, size=(2, 2, 0.01))
    rospy.sleep(0.5)
#############################################################
    group.set_num_planning_attempts(200)
    pose = group.get_current_pose(group.get_end_effector_link())
    group.set_path_constraints(None) 

##########################################################
    """way point """
##########################################################

    group.set_position_target([-1, 2, 0.2], group.get_end_effector_link())
    result = group.plan()
    # print(result.joint_trajectory.points[-1].positions)  

    # Move arm to avoid obstacle when printing 
    joint_goal = group.get_current_joint_values()
    joint_goal[4:] = result.joint_trajectory.points[-1].positions[4:]
    joint_goal[2] = pi/2
    joint_goal[3] = 2.9496
    group.go(joint_goal)

    pose = group.get_current_pose(group.get_end_effector_link())
    pose.pose.position.x = -1
    pose.pose.position.y = 2
    pose.pose.position.z = 0.2
    group.set_pose_target(pose)
    group.go()
    # Move the robot to the center of the striaght line to make sure Way point method can be executed
    # Way points
    waypoints = []

    wpose = base_group.get_current_pose().pose

    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = -1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x = 1
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = base_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    base_group.execute(plan)
###################################################################
    for i in range(3):


        rotation(90)
        constraints = Constraints()
        joint_constraint = JointConstraint()

        constraints.joint_constraints.append(JointConstraint(joint_name='arm_joint_1',
                                                             position = 2.95, tolerance_above=0.05,
                                                             tolerance_below=0.05, weight=1))

        # constraints.joint_constraints.append(JointConstraint(joint_name='z_joint',
        #                                                      position = pi, tolerance_above=0.05,
        #                                                      tolerance_below=0.05, weight=1))


        group.set_path_constraints(constraints)

        waypoints = []
        wpose = group.get_current_pose().pose

        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = 1
        wpose.position.y = 2
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = 1
        wpose.position.y = 4
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)       # jump_threshold
        group.execute(plan)
        group.set_path_constraints(None)

    #########################


        rotation(90)

        constraints = Constraints()
        joint_constraint = JointConstraint()

        constraints.joint_constraints.append(JointConstraint(joint_name='arm_joint_1',
                                                             position = 2.95, tolerance_above=0.05,
                                                             tolerance_below=0.05, weight=1))

        # constraints.joint_constraints.append(JointConstraint(joint_name='z_joint',
        #                                                      position = pi*3/2, tolerance_above=0.05,
        #                                                      tolerance_below=0.05, weight=1))


        group.set_path_constraints(constraints)


        waypoints = []

        wpose = group.get_current_pose().pose

        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = 1
        wpose.position.y = 4
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = -1
        wpose.position.y = 4
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
        group.execute(plan)
        group.set_path_constraints(None)

    ##########################
        rotation(90)
        constraints = Constraints()
        joint_constraint = JointConstraint()

        constraints.joint_constraints.append(JointConstraint(joint_name='arm_joint_1',
                                                             position = 2.95, tolerance_above=0.05,
                                                             tolerance_below=0.05, weight=1))

        # constraints.joint_constraints.append(JointConstraint(joint_name='z_joint',
        #                                                      position = pi*2, tolerance_above=0.05,
        #                                                      tolerance_below=0.05, weight=1))


        group.set_path_constraints(constraints)
        waypoints = []

        wpose = group.get_current_pose().pose

        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = -1
        wpose.position.y = 4
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = -1
        wpose.position.y = 2
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)          # jump_threshold
        group.execute(plan)
        group.set_path_constraints(None)


    ########################################################################
        rotation(90)

        ### Move the robot to initial point
        group.set_position_target([-1, 2, 0.2], group.get_end_effector_link())
        result = group.plan()
        # print(result.joint_trajectory.points[-1].positions)  

        # Move arm to avoid obstacle when printing 
        joint_goal = group.get_current_joint_values()
        joint_goal[4:] = result.joint_trajectory.points[-1].positions[4:]
        joint_goal[2] = pi/2
        joint_goal[3] = 2.9496
        group.go(joint_goal)

        pose = group.get_current_pose(group.get_end_effector_link())
        pose.pose.position.x = -1
        pose.pose.position.y = 2
        pose.pose.position.z = 0.2
        group.set_pose_target(pose)
        group.go()

        ### go straight line
        waypoints = []

        wpose = base_group.get_current_pose().pose

        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = -1
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = 1
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = base_group.compute_cartesian_path(
                                           waypoints,   # waypoints to follow
                                           0.01,        # eef_step
                                           0.0)         # jump_threshold
        base_group.execute(plan)

    print 'all finish'

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    rospy.init_node('move_group_python_interface')
    try:
        cw3_example_script()
    except rospy.ROSInterruptException:
        pass