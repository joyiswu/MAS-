#!/usr/bin/env python

import sys
import rospy
import copy
import actionlib
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume, RobotTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import PositionIKRequest
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from std_msgs.msg import Float32
import geometry_msgs.msg
# from moveit_commander.conversions import pose_to_list
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, atan2
from math import cos, sin
from sensor_msgs.msg import JointState
import numpy as np

def move_to_initial(goal):

    group.set_position_target([goal[0], goal[1], goal[2]], group.get_end_effector_link())
    while(1):
        result = group.plan()
        print 'number of points in trajectory:', len(result.joint_trajectory.points)

        # Threshold for short path
        if len(result.joint_trajectory.points) < 180 and len(result.joint_trajectory.points) > 0 : break

    return result

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
    global group, base_group, arm_group

    # request_prin = rospy.ServiceProxy('set_extruder_printing', SetBool)
    msg_prit = SetBoolRequest()

    request_fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('robot')
    arm_group = moveit_commander.MoveGroupCommander('arm')
    # gripeer_group = moveit_commander.MoveGroupCommander('gripper')
    base_group = moveit_commander.MoveGroupCommander('base')

    # Create a DisplayTrajectory publisher which is used later to publish trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
    extruder_publisher = rospy.Publisher('set_extruder_rate',
                                               Float32,
                                               queue_size=20)

    msg_extrude = Float32()
    msg_extrude = 5.0
    extruder_publisher.publish(msg_extrude)

    group.allow_replanning(True)
    group.allow_looking(True)
    # Set the number of planning attempts to find better solution 
    group.set_num_planning_attempts(10)
################################################
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
    box_pose.pose.position.x = -0.75
    box_pose.pose.position.y = 3.5
    box_pose.pose.position.z = 0.2
    box_one = "box_1"
    scene.add_box(box_one, box_pose, size=(0.5, 0.5, 0.5))
    rospy.sleep(0.5)

    rospy.sleep(0.5)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "odom"
    box_pose.pose.position.x = -0
    box_pose.pose.position.y = 2.7
    box_pose.pose.position.z = 0.2
    box_two = "box_2"
    scene.add_box(box_two, box_pose, size=(0.5, 0.5, 0.5))
    rospy.sleep(0.5)

    rospy.sleep(0.5)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "odom"
    box_pose.pose.position.x = 0.75
    box_pose.pose.position.y = 3.5
    box_pose.pose.position.z = 0.2
    box_three = "box_3"
    scene.add_box(box_three, box_pose, size=(0.5, 0.5, 0.5))
    rospy.sleep(0.5)

#############################################################
    point_list = []
    for i in range(11):
        point_list.append((i * 0.2 -1, 3, 0.1))
    print(point_list)
#############################################################

########################################################################
##Add constrain
    pose = group.get_current_pose(group.get_end_effector_link())
    constraints = Constraints()
    # orientation_constraint = OrientationConstraint()
    # orientation_constraint.header = pose.header
    # orientation_constraint.link_name = group.get_end_effector_link()
    # orientation_constraint.orientation = pose.pose.orientation
    # orientation_constraint.absolute_x_axis_tolerance = 0.1
    # orientation_constraint.absolute_y_axis_tolerance = 0.1
    # orientation_constraint.absolute_z_axis_tolerance = 2*pi
    # current_orientation_list = [pose.pose.orientation.x,
    #                             pose.pose.orientation.y,
    #                             pose.pose.orientation.z,
    #                             pose.pose.orientation.w]

    # # get euler angle from quaternion
    # (roll, pitch, yaw) = euler_from_quaternion(current_orientation_list)
    # pitch = pi
    # roll = 0
    # orientation_constraint.weight = 1

    # [orientation_constraint.orientation.x, orientation_constraint.orientation.y, orientation_constraint.orientation.z, orientation_constraint.orientation.w] = \
    #     quaternion_from_euler(roll, pitch, yaw)

    # constraints.orientation_constraints.append(orientation_constraint) 
    # group.set_path_constraints(constraints) 

    #### joint constraints
    joint_constraint = JointConstraint()
    joint_constraint.joint_name = 'arm_joint_1'
    joint_constraint.position = 169*pi/180
    joint_constraint.tolerance_above = 30*pi/180
    joint_constraint.tolerance_below = 30*pi/180
    joint_constraint.weight = 1

    constraints.joint_constraints.append(joint_constraint)
    group.set_path_constraints(constraints)

#######################################################################


    while len(point_list) > 1:

        # Move the robot point to first point and find the height
        initial_plan = move_to_initial(point_list[1])
        joint_goal = group.get_current_joint_values()
        head = initial_plan.joint_trajectory.header
        robot_state = robot.get_current_state()
        # print(robot.get_current_state().joint_state.position)
        robot_state.joint_state.position = tuple(initial_plan.joint_trajectory.points[-1].positions) + \
                                           tuple(robot_state.joint_state.position[7:])


        resp = request_fk(head, [group.get_end_effector_link()], robot_state)

        current_pose = group.get_current_pose().pose
        current_pose.orientation = resp.pose_stamped[0].pose.orientation
        (current_pose.position.x, current_pose.position.y, current_pose.position.z) = point_list[0]

        group.set_pose_target(current_pose)
        group.go()

        # Move the robot to the center of the striaght line to make sure Way point method can be executed
        # Way points
        waypoints = []

        wpose = group.get_current_pose().pose

        # Add the current pose to make sure the path is smooth
        waypoints.append(copy.deepcopy(wpose))

        success_num = 0

        for point_num in range(len(point_list)):

            (wpose.position.x, wpose.position.y, wpose.position.z) = point_list[point_num]
            waypoints.append(copy.deepcopy(wpose))

            (plan, fraction) = group.compute_cartesian_path(
                                               waypoints,   # waypoints to follow
                                               0.01,        # eef_step
                                               0.0)         # jump_threshold
            if fraction == 1:
                success_num += 1
                execute_plan = plan
                if success_num == len(point_list): group.execute(execute_plan)

            elif success_num == 0:
                break
            else:
                # execute success plan
                msg_prit.data = True
                group.execute(execute_plan)
                break
        msg_prit.data = False

        # Delete the points what already execute
        if success_num > 0: 
            del(point_list[0:success_num-1])

            # Add obstacle after printing (need to revise)

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