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
from actionlib_msgs.msg import GoalStatusArray
import moveit_python
from obstacle_command import Scene_obstacle
from visualization_msgs.msg import Marker
import re
import matplotlib.pyplot as plt
import datetime

class arm_base_control:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()

        # Init moveit group
        self.group = moveit_commander.MoveGroupCommander('robot')
        self.arm_group = moveit_commander.MoveGroupCommander('arm')
        self.base_group = moveit_commander.MoveGroupCommander('base')

        self.scene = moveit_commander.PlanningSceneInterface()

        self.sce = moveit_python.PlanningSceneInterface('odom')
        self.pub_co = rospy.Publisher('collision_object', CollisionObject, queue_size=100)

        self.msg_print = SetBoolRequest()

        self.request_fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)

        self.pub_co = rospy.Publisher('collision_object', CollisionObject, queue_size=100)

        sub_waypoint_status = rospy.Subscriber('execute_trajectory/status', GoalStatusArray, self.waypoint_execution_cb)
        sub_movegroup_status = rospy.Subscriber('execute_trajectory/status', GoalStatusArray, self.move_group_execution_cb)
        # sub_movegroup_status = rospy.Subscriber('move_group/status', GoalStatusArray, self.move_group_execution_cb)
        rospy.Subscriber("joint_states", JointState, self.further_ob_printing)
        # rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        # # Initialize extruder
        # print_request = rospy.ServiceProxy('/'+current_robot_ns+'/set_extruder_printing', SetBool)
        # self.extruder_publisher = rospy.Publisher('/'+current_robot_ns+'/set_extruder_rate',Float32,queue_size=20)
        
        # switch the extruder ON, control via set_extruder_rate
        msg_print = SetBoolRequest()
        msg_print.data=True

        self.re_position_x = []
        self.re_position_y = [] 

    
        self.waypoint_execution_status = 0
        self.move_group_execution_status = 0
        self.further_printing_number = 0
        self.pre_further_printing_number = 0

        # initial printing number 
        self._printing_number = 0
        self._further_printing_number = 0
        self.future_printing_status = False

        self.current_printing_pose = None
        self.previous_printing_pose = None

        self.target_list = None

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
        self.pub_rviz_marker = rospy.Publisher('/visualization_marker', Marker, queue_size=100)
        self.remove_all_rviz_marker()
        self.printing_number_rviz = 0

    # def joint_callback(self, data):

    #     current_ee_pose = self.group.get_current_pose().pose
    #     current_base_pose = self.base_group.get_current_pose().pose

    #     self.base_position_x.append(round(current_base_pose.position.x, 4))
    #     self.base_position_y.append(round(current_base_pose.position.y, 4))
    #     self.eef_position_x.append(round(current_ee_pose.position.x, 4))
    #     self.eef_position_y.append(round(current_ee_pose.position.y, 4))

    def remove_all_rviz_marker(self):
        marker = Marker()
        marker.header.frame_id = "/odom"
        marker.action = marker.DELETEALL
        # publish a few times to ensure it successfully received
        for i in range(5):
            self.pub_rviz_marker.publish(marker)
            rospy.sleep(0.1)

    def rviz_visualise_marker(self, point_list):
        triple_points = []

        for (x,y,z) in point_list:
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            triple_points.append(p)

        marker = Marker()
        marker.header.frame_id = "/odom"
        # marker.ns = 'robot' + str(self.robot_id)
        marker.id = self.printing_number_rviz
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.pose.orientation.w = 1

        marker.points = triple_points
        marker.lifetime = rospy.Duration(0) # 0 means forever
        marker.scale.x = 0.05 # linestrip only needs scale.x
        marker.color.a = 1.0 # opacity value
        marker.color.r = 1.0 # rgb value

        self.printing_number_rviz += 1

        # publish a few times to ensure it successfully received
        for i in range(5):
            self.pub_rviz_marker.publish(marker)
            rospy.sleep(0.1)
    # Demo 1 scene
    def add_three_box_obstacle(self):
        ###Add obstacle
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
    def printing_visualize(self, start_point, end_point, name = 'obstacle'):
        # print(start_point)
        rospy.sleep(0.05)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "odom"
        box_pose.pose.position.x = (end_point[0] + start_point[0]) / 2.000000
        box_pose.pose.position.y = (end_point[1] + start_point[1]) / 2.000000
        box_pose.pose.position.z = (end_point[2] + start_point[2]) / 2.000000 - 0.05

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


        self._printing_number += 1
        box_name = str(name) + str(self._printing_number)
        self.pub_co.publish(self.make_box(box_name, box_pose, size=(length, 0.01, 0.01)))
        rospy.sleep(0.05)

    def print_list_visualize(self, way_points, name = 'obstacle'):

        ob_name = []
        for point in way_points:
            ob_name.append(name)
        if len(way_points)>0:
            way_points = [way_points[0]] + way_points
            differential = map(self.printing_visualize, way_points[:-1], way_points[1:], ob_name)

    def future_visualize(self, start_point, end_point):
        # print(start_point)
        rospy.sleep(0.05)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "odom"
        box_pose.pose.position.x = (end_point[0] + start_point[0]) / 2.000000
        box_pose.pose.position.y = (end_point[1] + start_point[1]) / 2.000000
        box_pose.pose.position.z = (end_point[2] + start_point[2]) / 2.000000 - 0.05

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


        self._further_printing_number += 1
        if self._further_printing_number == 5: self._further_printing_number = 0
        box_name = 'future_point' + str(self._further_printing_number)
        self.pub_co.publish(self.make_box(box_name, box_pose, size=(length, 0.01, 0.01)))


        rospy.sleep(0.05)

    def print_future_visualize(self, way_points, index, step = 8, length = 5, status = True, point_num = 1):

        # There is no future obstacle here
        if status == False:
            if len(way_points) > index + step:
                # Print full length future obstacles
                if len(way_points) > index + step + length:
                    differential = map(self.future_visualize, 
                                       way_points[index+step:index+step+length], \
                                       way_points[index+step+1:index+step+length+1])
                # Print partial future obstacle
                else:
                    differential = map(self.future_visualize, way_points[index+step:-1], way_points[index+step+1:])
        # There already have future obstacle            
        else:
            if len(way_points) > index + step:
                # Add obstacle
                if len(way_points) > index + step + length:
                    differential = map(self.future_visualize, 
                                       way_points[index+length+step-point_num:index+length+step-1], \
                                       way_points[index+length+step-point_num+1:index+length+step])
                # # Remove obstacle
                # else:
                #     loop_num = step + length + index - len(way_points) -1 
                #     for i in range(loop_num):
                #         self.future_visualize(way_points[-1], way_points[-1])

    def get_circle_point(self, circle_center, radius, height = 0.1, degree_resolution = 10):

        point_list = []
        degree = 0

        while(degree < 360):

            theta = degree * pi / 180

            point_list.append(( circle_center[0] + radius * cos(theta), \
                                circle_center[1] + radius * sin(theta), height))

            degree = degree + degree_resolution

        point_list.append(( circle_center[0] + radius * cos(2*pi), \
                            circle_center[1] + radius * sin(2*pi), height))

        return point_list


    # Sampling the points in straight
    def straight_line_sample(self, start_point, end_point, resolution = 10, height = 0.1):
        point_list = []
        point_list.append((start_point[0], start_point[1], height))

        # Calculate the distance between starting point and ending point
        distance = np.sqrt(pow(start_point[0] - end_point[0], 2) + pow(start_point[1] - end_point[1], 2))

        # Calculate the number of the waypoints
        num_point = int(distance * resolution)

        if num_point > 1:

            # Add waypoints
            for i in range(1, num_point):
                x = start_point[0] + i * ((end_point[0] - start_point[0]) / (distance * resolution))
                y = start_point[1] + i * ((end_point[1] - start_point[1]) / (distance * resolution))
                point_list.append((x, y, height))

        point_list.append((end_point[0], end_point[1], height))

        return point_list

    def waypoint_execution_cb(self,msg):
        if len(msg.status_list)>0:
            self.waypoint_execution_status = msg.status_list[-1].status
            # print self.waypoint_execution_status

    def move_group_execution_cb(self,msg):
        # Add current printing obstacle
        if self.msg_print.data == True:
            current_ee_pose = self.group.get_current_pose().pose
            self.current_printing_pose = (current_ee_pose.position.x, current_ee_pose.position.y, current_ee_pose.position.z)
            
            self.re_position_x.append(current_ee_pose.position.x)
            self.re_position_y.append(current_ee_pose.position.y)

            if self.current_printing_pose and self.previous_printing_pose:
                self.printing_visualize(self.current_printing_pose, self.previous_printing_pose, name = 'printing point')

            # Update previous printing point
            self.previous_printing_pose = self.current_printing_pose



    def further_ob_printing(self,msg):
        if self.future_printing_status:

            current_future_ob_list = self.sce.getKnownCollisionObjects()
            future_printing_already = False

            for point_name in current_future_ob_list:
                result = re.match('future_ob', point_name)
                if result:
                    future_printing_already = True
                    break
               
            if self.further_printing_number == 0:
                self.print_future_visualize(
                    self.target_list, 
                    self.further_printing_number, 
                    status = future_printing_already)

            else:

                self.print_future_visualize(
                    self.target_list, 
                    self.further_printing_number, 
                    status = future_printing_already, 
                    point_num = self.further_printing_number - self.pre_further_printing_number)
            self.pre_further_printing_number = self.further_printing_number

    def move_to_initial(self, goal):

        self.group.set_position_target([goal[0], goal[1], goal[2]], self.group.get_end_effector_link())
        while(1):
            result = self.group.plan()
            # print 'number of points in trajectory:', len(result.joint_trajectory.points)

            # Threshold for short path
            if len(result.joint_trajectory.points) < 180 and len(result.joint_trajectory.points) > 0 : break

        return result

    def check_executed_waypoint_index(self, success_planned_waypoints, current_ee_position):
        found_status = 0
        waypoint_index = len(success_planned_waypoints) -1
        epsilon = 1e-6
        # both input arguments are in np.array format, 2 dimensions only (x,y axis)
        for waypoint_index in range(len(success_planned_waypoints)-1):
            previous_waypoint = success_planned_waypoints[waypoint_index]
            current_waypoint = success_planned_waypoints[waypoint_index+1]

            # cross(b-a,c-a)
            cross_prod = np.cross(current_waypoint - previous_waypoint,
                                  current_ee_position - previous_waypoint)

            # print 'cross prod', cross_prod, waypoint_index

            # if is aligned or parallel
            if abs(cross_prod) <= epsilon:
                # check if current position is between these 2 waypoints
                distance_between_points = np.linalg.norm(current_waypoint-previous_waypoint)
                dotprod = np.dot(current_waypoint - previous_waypoint,
                                 current_ee_position - previous_waypoint)

                if 0 < dotprod < pow(distance_between_points,2):
                    # exit the loop, since the index has been found.
                    found_status = 1
                    break

        if found_status == 0:
            # unable to find any matching waypoint
            # require further debug especially hardware execution
            # waypoint_index = -1
            waypoint_index = 0 

        return waypoint_index

    def print_pointlist(self, point_list, future_print_status = False):

        startime = datetime.datetime.now()

        # Save original points list
        full_point_list = copy.deepcopy(point_list)

        full_point_array = np.delete(np.array(full_point_list), 2, axis=1)

        self.target_list = full_point_list

        if future_print_status: self.future_printing_status = True

        # Constraints
        pose = self.group.get_current_pose(self.group.get_end_effector_link())
        constraints = Constraints()
        # joint constraints
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'arm_joint_1'
        joint_constraint.position = 169*pi/180
        joint_constraint.tolerance_above = 30*pi/180
        joint_constraint.tolerance_below = 30*pi/180
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)

        joint_constraint = JointConstraint()
        joint_constraint.joint_name = 'arm_joint_4'
        joint_constraint.position = 150*pi/180
        joint_constraint.tolerance_above = 30*pi/180
        joint_constraint.tolerance_below = 30*pi/180
        joint_constraint.weight = 1.0
        constraints.joint_constraints.append(joint_constraint)
        self.group.set_path_constraints(constraints)

        # Orientation constrains
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose.header
        orientation_constraint.link_name = self.group.get_end_effector_link()
        orientation_constraint.orientation = pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 2*pi
        orientation_constraint.absolute_y_axis_tolerance = 2*pi
        orientation_constraint.absolute_z_axis_tolerance = 2*pi
        orientation_constraint.weight = 1.0

        constraints.orientation_constraints.append(orientation_constraint)

        # Record how many points has already finished
        finsih_num = 0
        print_num = 0
        index_check = 0
        while len(point_list) > 0:

            print('New Plan, points left:', len(point_list), point_list)

            # Move the robot point to first point and find the height
            if len(point_list) > 1:
                initial_plan = self.move_to_initial(point_list[1])
            else:
                initial_plan = self.move_to_initial(point_list[0])
            # joint_goal = self.group.get_current_joint_values()
            head = initial_plan.joint_trajectory.header
            robot_state = self.robot.get_current_state()
            # print(robot_state.joint_state)
            robot_state.joint_state.position = tuple(initial_plan.joint_trajectory.points[-1].positions) + \
                                               tuple(robot_state.joint_state.position[7:]) # the joints for the wheel


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
                    waypoints,  # waypoints to follow
                    0.01,        # eef_step
                    0.00,
                    path_constraints = constraints)

                print'Adding the first planing point, and fraction is', fraction
                executing_state = 0
                if fraction == 1:
                    success_num += 1
                    execute_plan = plan
                    if success_num == len(point_list):
                        self.group.execute(execute_plan, wait=False)
                        self.msg_print.data = True
                        executing_state = 1
                        success_num += 1

                elif success_num == 0:
                    break
                else:
                    # execute success plan
                    self.msg_print.data = True
                    self.group.execute(execute_plan, wait=False)
                    executing_state = 1
                    break

            ## 2nd loop
            ## always re-plan and check for obstacle while executing waypoints

            executed_waypoint_index = 0 # initial value of nothing

            success_point_list = point_list[:success_num]
            print('when plan success, move_group_status:', self.move_group_execution_status, 'success_plan_number:', success_num)

            if executing_state == 1 :
                success_planned_waypoint_array = np.delete(np.array(point_list[:success_num]), 2, axis=1)
                # print 'success planned waypoint\n', success_planned_waypoint_array
                print 'status', self.waypoint_execution_status

                while self.waypoint_execution_status != 3:

                    if point_list == []: break
                    
                    if self.waypoint_execution_status == 4:
                        # aborted state
                        print 'stop and abort waypoint execution'
                        self.msg_print.data = False
                        self.group.stop()
                        executing_state = 0
                        break

                    current_ee_pose = self.group.get_current_pose().pose
                    current_ee_position_array = np.array([current_ee_pose.position.x,
                                                          current_ee_pose.position.y])

                    executed_waypoint_index = self.check_executed_waypoint_index(success_planned_waypoint_array, current_ee_position_array)
                    # print 'executed latest index', executed_waypoint_index

                    index_check = self.check_executed_waypoint_index(full_point_array, current_ee_position_array)
                    self.further_printing_number = index_check
                    print 'index:', index_check, 'way_point index', executed_waypoint_index

                    if future_print_status == True:

                        # Check for enclosure
                        self.base_group.set_position_target([0, 0, 0.05], self.base_group.get_end_effector_link())
                        result = self.base_group.plan()
                        self.base_group.clear_pose_targets()

                        if len(result.joint_trajectory.points) == 0: 
                            print('Check enclosure failed')

                        else: print('Check enclosure successful')

                    ## Replan to check for dynamic obstacle
                    waypoints = []
                    # Add the current pose to make sure the path is smooth, get latest pose
                    current_ee_pose = self.group.get_current_pose().pose
                    waypoints.append(copy.deepcopy(current_ee_pose))

                    # discard the executed waypoints
                    new_point_list = point_list[executed_waypoint_index:success_num]

                    for k in new_point_list:
                        (current_ee_pose.position.x, current_ee_pose.position.y, current_ee_pose.position.z) = k
                        waypoints.append(copy.deepcopy(current_ee_pose))

                    (plan2, fraction2) = self.group.compute_cartesian_path(
                                                waypoints,  # waypoints to follow
                                                0.01,        # eef_step
                                                0.00,
                                                path_constraints = constraints)
                    print 'Dynamic check fraction:', fraction2
                    if fraction2 < 1:
                        ## new obstacle appear
                        # print 'executed latest index', executed_waypoint_index
                        # print 'fraction value', fraction,'\n'
                        print 'new obstacle appeared to be in the path'
                        self.group.stop()
                        self.msg_print.data = False
                        executing_state = 0
                        break

                rospy.sleep(2)

                print 'status:', self.waypoint_execution_status, 'executed_index:',executed_waypoint_index, 'success_num:', success_num

                if self.waypoint_execution_status == 3:
                    # waypoint successfully printed
                    # self.print_list_visualize(point_list[:success_num])
                    self.rviz_visualise_marker(point_list[:success_num])
                    del(point_list[:success_num-1])

                elif self.waypoint_execution_status == 2 or 4:
                    # state 2 = preempted, state 4 = aborted.
                    # only printed partial waypoint
                    # self.print_list_visualize(point_list[:executed_waypoint_index+1])
                    if executed_waypoint_index > 0: # at index 0, it might have not print the point 0-1 edge successfully.
                        self.rviz_visualise_marker(point_list[:executed_waypoint_index+1])
                        del(point_list[:executed_waypoint_index]) # delete up till whatever is executed

            self.msg_print.data = False
            
            if point_list == []: rospy.sleep(2)
            self.group.stop()
            # Delete the points what already execute
            # if success_num > 0:
                # Add obstacle after printing (need to revise)

        self.group.set_path_constraints(None)

        finshtime = datetime.datetime.now()
        
        print(finshtime-startime).seconds
        full_point_list_x = [base[0] for base in full_point_list]
        full_point_list_y = [base[1] for base in full_point_list]


        plt3 = plt.figure("Printing result", figsize=(6, 6))
        plt.plot(self.re_position_x ,self.re_position_y, 'b')
        plt.plot(full_point_list_x, full_point_list_y, 'ro')
        plt.xlim(min(full_point_list_x) - 0.1, max(full_point_list_x) + 0.1)
        plt.ylim(min(full_point_list_y) - 0.1, max(full_point_list_y) + 0.1)
        plt.legend(['Printing result', 'ground truth'], fontsize=8, bbox_to_anchor=(1.0, 1))
        plt.xlabel("X axis (m)")
        plt.ylabel("y axis (m)")
        plt.title('Printing result')

        plt.show()
        plt3.savefig('result.png', dpi=plt3.dpi)

        print('All finish, time:', (finshtime-startime).seconds)