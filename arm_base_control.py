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
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, atan2
from math import cos, sin
from sensor_msgs.msg import JointState
import numpy as np


class arm_base_printing:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('arm_base_printing')

        self.robot = moveit_commander.RobotCommander()

        # Initialize moveit scene interface (woeld surrounding the robot)
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group = moveit_commander.MoveGroupCommander('robot')

        self.msg_prit = SetBoolRequest()

        self.request_fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)


        self.group.allow_looking(1)
        self.group.allow_replanning(1)
        self.group.set_planning_time(10)



        # Initialize extruder

        extruder_publisher = rospy.Publisher('set_extruder_rate',
                                                   Float32,
                                                   queue_size=20)
        msg_extrude = Float32()
        msg_extrude = 5.0
        extruder_publisher.publish(msg_extrude)

        self.add_three_box_obstacle()
        point_list = []
        for i in range(11):
            point_list.append((i * 0.2 -1, 3, 0.1))
        print(point_list)
        self.print_pointlist(point_list)


    # Sampling the points in straight 
    def straight_line_sample(self, start_point, end_point, resolution = 10):
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

    def add_three_box_obstacle(self):
    ###Add obstacle
        rospy.sleep(0.5)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "odom"
        box_pose.pose.position.x = 0
        box_pose.pose.position.y = 3
        box_pose.pose.position.z = 0.01
        box_name = "wall"
        self.scene.add_box(box_name, box_pose, size=(2, 0.01, 0.01))
        rospy.sleep(0.5)

        rospy.sleep(0.5)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "odom"
        box_pose.pose.position.x = -0.75
        box_pose.pose.position.y = 3.5
        box_pose.pose.position.z = 0.2
        box_one = "box_1"
        self.scene.add_box(box_one, box_pose, size=(0.5, 0.5, 0.5))
        rospy.sleep(0.5)

        rospy.sleep(0.5)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "odom"
        box_pose.pose.position.x = -0
        box_pose.pose.position.y = 2.7
        box_pose.pose.position.z = 0.2
        box_two = "box_2"
        self.scene.add_box(box_two, box_pose, size=(0.5, 0.5, 0.5))
        rospy.sleep(0.5)

        rospy.sleep(0.5)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "odom"
        box_pose.pose.position.x = 0.75
        box_pose.pose.position.y = 3.5
        box_pose.pose.position.z = 0.2
        box_three = "box_3"
        self.scene.add_box(box_three, box_pose, size=(0.5, 0.5, 0.5))
        rospy.sleep(0.5)

    def move_to_initial(self, goal):

        self.group.set_position_target([goal[0], goal[1], goal[2]], self.group.get_end_effector_link())
        while(1):
            result = self.group.plan()
            print 'number of points in trajectory:', len(result.joint_trajectory.points)

            # Threshold for short path
            if len(result.joint_trajectory.points) < 180 and len(result.joint_trajectory.points) > 0 : break

        return result

    def print_pointlist(self, point_list):
    ##Add constrain
        pose = self.group.get_current_pose(self.group.get_end_effector_link())
        constraints = Constraints()


        #### joint constraints
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'arm_joint_1'
        joint_constraint.position = 169*pi/180
        joint_constraint.tolerance_above = 30*pi/180
        joint_constraint.tolerance_below = 30*pi/180
        joint_constraint.weight = 1

        constraints.joint_constraints.append(joint_constraint)
        self.group.set_path_constraints(constraints)


        while len(point_list) > 1:

            # Move the robot point to first point and find the height
            initial_plan = self.move_to_initial(point_list[1])
            joint_goal = self.group.get_current_joint_values()
            head = initial_plan.joint_trajectory.header
            robot_state = self.robot.get_current_state()
            # print(robot.get_current_state().joint_state.position)
            robot_state.joint_state.position = tuple(initial_plan.joint_trajectory.points[-1].positions) + \
                                               tuple(robot_state.joint_state.position[7:])


            resp = self.request_fk(head, [self.group.get_end_effector_link()], robot_state)

            current_pose = self.group.get_current_pose().pose
            current_pose.orientation = resp.pose_stamped[0].pose.orientation
            (current_pose.position.x, current_pose.position.y, current_pose.position.z) = point_list[0]

            self.group.set_pose_target(current_pose)
            self.group.go()

            # Move the robot to the center of the striaght line to make sure Way point method can be executed
            # Way points
            waypoints = []

            wpose = self.group.get_current_pose().pose

            # Add the current pose to make sure the path is smooth
            waypoints.append(copy.deepcopy(wpose))

            success_num = 0

            for point_num in range(len(point_list)):

                (wpose.position.x, wpose.position.y, wpose.position.z) = point_list[point_num]
                waypoints.append(copy.deepcopy(wpose))

                (plan, fraction) = self.group.compute_cartesian_path(
                                                   waypoints,   # waypoints to follow
                                                   0.01,        # eef_step
                                                   0.0)         # jump_threshold
                if fraction == 1:
                    success_num += 1
                    execute_plan = plan
                    if success_num == len(point_list): self.group.execute(execute_plan)

                elif success_num == 0:
                    break
                else:
                    # execute success plan
                    self.msg_prit.data = True
                    self.group.execute(execute_plan)
                    break
            self.msg_prit.data = False

            # Delete the points what already execute
            if success_num > 0: 
                del(point_list[0:success_num-1])

                # Add obstacle after printing (need to revise)

        self.group.set_path_constraints(None)
        print 'all finish'

if __name__ == '__main__':
    try:
        arm_base_printing()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        pass
