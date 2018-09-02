#!/usr/bin/env python
import sys
import rospy
import copy
import actionlib
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume, RobotTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, CollisionObject
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
from std_srvs.srv import SetBool, SetBoolResponse, SetBoolRequest
from std_msgs.msg import Float32
import geometry_msgs.msg
from geometry_msgs.msg import * 
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, atan2, cos, sin, tan, sqrt
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

        self.arm_group = moveit_commander.MoveGroupCommander('arm')

        self.msg_prit = SetBoolRequest()

        self.request_fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)

        self.pub_co = rospy.Publisher('collision_object', CollisionObject, queue_size=100)

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

        self.printing_number = 0

        # self.add_three_box_obstacle()
        # point_list = []

        # for i in range(11):
        #     point_list.append((i * 0.2 -1, 3, 0.1))
        # print(point_list)

        point_list = self.get_circle_point((0,0), 1)
        self.print_pointlist(point_list)
        # self.move_arm()
        # self.printing_visualize((0,1,0),(1,0,0))

    def get_way_point_differential(self, way_points):
        if len(way_points)>0:
            differential = map(self.get_two_point_differential, way_points[:-1], way_points[1:])
        return differential

    def get_two_point_differential(self, start_point, end_point):
        if (end_point[0] - start_point[0]) == 0: 
            return 'inf'
        else:
            return atan2((end_point[1] - start_point[1]),(end_point[0] - start_point[0]))

    def get_circle_point(self, circle_center, radius, degree_resolution = 10):

        point_list = []
        degree = 0

        while(degree < 360):

            theta = degree * pi / 180

            # if theta > pi: theta -= 2 * pi
            # if theta < pi: theta += 2 * pi   

            point_list.append(( circle_center[0] + radius * cos(theta), \
                                circle_center[1] + radius * sin(theta), 0.1))

            degree = degree + degree_resolution

        point_list.append(( circle_center[0] + radius * cos(2*pi), \
                            circle_center[1] + radius * sin(2*pi), 0.1))

        return point_list


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

    def make_box(self, name, pose, size = (0.5, 0.5, 0.5)):
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)
        co.primitives = [box]
        co.primitive_poses = [pose.pose]
        return co

    # Add a line obstacle between two points (only concern about same height)
    def printing_visualize(self, start_point, end_point):
        rospy.sleep(0.3)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "odom"
        box_pose.pose.position.x = (end_point[0] + start_point[0]) / 2.000000
        box_pose.pose.position.y = (end_point[1] + start_point[1]) / 2.000000
        box_pose.pose.position.z = (end_point[2] + start_point[2]) / 2.000000

        (roll, pitch, yaw) = euler_from_quaternion([box_pose.pose.orientation.x,
                                                    box_pose.pose.orientation.y,
                                                    box_pose.pose.orientation.z,
                                                    box_pose.pose.orientation.w])

        yaw = atan2((end_point[1] - start_point[1]),(end_point[0] - start_point[0]))

        [box_pose.pose.orientation.x, \
         box_pose.pose.orientation.y, \
         box_pose.pose.orientation.z, \
         box_pose.pose.orientation.w] = \
         quaternion_from_euler(roll, pitch, yaw )

        length = sqrt(pow((end_point[0] - start_point[0]), 2) + \
                      pow((end_point[1] - start_point[1]), 2) + \
                      pow((end_point[2] - start_point[2]), 2))


        self.printing_number += 1 
        box_name = "printing_point" + str(self.printing_number)
        self.pub_co.publish(self.make_box(box_name, box_pose, size=(length, 0.01, 0.01)))
        rospy.sleep(0.3)

    def print_list_visualize(self, way_points):
        if len(way_points)>0:
            differential = map(self.printing_visualize, way_points[:-1], way_points[1:])

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

    def move_arm(self):

        arm_pose = self.arm_group.get_current_pose(self.arm_group.get_end_effector_link())
        print(arm_pose)

        waypoints = []

        scale = 1
        wpose = self.arm_group.get_current_pose().pose

        wpose.position.z = 0.1

        waypoints.append(copy.deepcopy(wpose))

        i = 0

        while(i>10):

            wpose.position.x = wpose.position.x - 0.005
            wpose.position.y = wpose.position.x + 0.005
            waypoints.append(copy.deepcopy(wpose))

        # wpose.position.x = wpose.position.x + 0.1
        # wpose.position.y = wpose.position.x - 0.1
        # waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
        print(fraction)

        self.arm_group.execute(plan, wait = True)


    def print_pointlist(self, point_list):

        # Get differential of way point list
        differential = self.get_way_point_differential(point_list)
        differential_num = [0]
        differential_num = differential_num + differential

        eef_rotation_change = [0, 0]
        for i in range(2, len(differential_num)):
            eef_rotation_change.append(differential_num[i] - differential_num[i-1])

 

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

                (roll, pitch, yaw) = euler_from_quaternion([wpose.orientation.x,
                                                            wpose.orientation.y,
                                                            wpose.orientation.z,
                                                            wpose.orientation.w])

                [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w] = \
                    quaternion_from_euler(roll, pitch, yaw + eef_rotation_change[point_num])
                print(yaw + eef_rotation_change[point_num])
                # [wpose.orientation.x, wpose.orientation.y, wpose.orientation.z, wpose.orientation.w] = \
                #     quaternion_from_euler(roll, pitch, yaw + 0.1744444)


                waypoints.append(copy.deepcopy(wpose))

                (plan, fraction) = self.group.compute_cartesian_path(
                                                   waypoints,  # waypoints to follow
                                                   0.01,        # eef_step
                                                   0.00)         # jump_threshold
                if fraction == 1:
                    success_num += 1
                    execute_plan = plan
                    if success_num == len(point_list): 
                        self.group.execute(execute_plan)
                        self.print_list_visualize(point_list)

                elif success_num == 0:
                    break
                else:
                    # execute success plan
                    self.msg_prit.data = True
                    self.group.execute(execute_plan)
                    self.print_list_visualize(point_list[:success_num])
                    break
            self.msg_prit.data = False

            # Delete the points what already execute
            if success_num > 0: 
                del(point_list[0:success_num-1])
                del(eef_rotation_change[0:success_num-1])
                # Add obstacle after printing (need to revise)

        self.group.set_path_constraints(None)
        print 'all finish'

if __name__ == '__main__':
    try:
        arm_base_printing()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        pass
