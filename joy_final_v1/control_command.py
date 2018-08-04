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

class arm_base_control:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()

        # Init moveit group
        self.group = moveit_commander.MoveGroupCommander('robot')
        self.arm_group = moveit_commander.MoveGroupCommander('arm')

        self.scene = Scene_obstacle()

        self.msg_print = SetBoolRequest()

        self.request_fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)

        self.pub_co = rospy.Publisher('collision_object', CollisionObject, queue_size=100)

        sub_waypoint_status = rospy.Subscriber('execute_trajectory/status', GoalStatusArray, self.waypoint_execution_cb)
        
        # check_inside = rospy.Subscriber("joint_states", JointState, self.check_inside_callback)

        self.waypoint_execution_status = 0

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


    def waypoint_execution_cb(self,msg):
        if len(msg.status_list)>0:
            self.waypoint_execution_status = msg.status_list[-1].status
            # print self.waypoint_execution_status

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

        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold
        print(fraction)

        self.arm_group.execute(plan, wait = True)

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

    def print_pointlist(self, point_list):

        # Save original points list
        all_point_list = point_list

        pose = self.group.get_current_pose(self.group.get_end_effector_link())
        constraints = Constraints()
        last_ee_pose = point_list[0]
        #### joint constraints
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
        while len(point_list) > 1:

            # Move the robot point to first point and find the height
            initial_plan = self.move_to_initial(point_list[1])
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

                executing_state = 0
                if fraction == 1:
                    success_num += 1
                    execute_plan = plan
                    if success_num == len(point_list):
                        self.group.execute(execute_plan, wait=False)
                        executing_state = 1

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

            if executing_state == 1:
                success_planned_waypoint_array = np.delete(np.array(point_list[:success_num]), 2, axis=1)
                print 'success planned waypoint\n', success_planned_waypoint_array
                print 'status', self.waypoint_execution_status


                while self.waypoint_execution_status != 3:

                    current_ee_pose = self.group.get_current_pose().pose
                    current_ee_position_array = np.array([current_ee_pose.position.x,
                                                          current_ee_pose.position.y])
       

                    self.scene.printing_visualize(last_ee_pose, (current_ee_pose.position.x, current_ee_pose.position.y, current_ee_pose.position.z), 
                                                  name = 'printing point')

                    last_ee_pose = (current_ee_pose.position.x, current_ee_pose.position.y, current_ee_pose.position.z)
                    executed_waypoint_index = self.check_executed_waypoint_index(success_planned_waypoint_array, current_ee_position_array)

                    # print 'last_ee', (last_ee_pose.position.x,last_ee_pose.position.y),     \
                    #       'current_ee', (current_ee_pose.position.x,current_ee_pose.position.y),   \
                    #       'executed latest index', executed_waypoint_index
                    # print(executed_waypoint_index)
                    # print 'current number', finsih_num + executed_waypoint_index, 'printing number', print_num
                    
                    # if finsih_num + executed_waypoint_index - print_num <= 2 and finsih_num + executed_waypoint_index - print_num >=-2:
                    #     self.print_future_visualize(all_point_list, print_num)
                    #     print_num = finsih_num + executed_waypoint_index + 1
                        # # print(success_planned_waypoint_array.size)
                        # print('printing')
                        # print(finsih_num + executed_waypoint_index)
                        # next_future_ob_index = finsih_num + executed_waypoint_index
                        # if next_future_ob_index + 9 < len(all_point_list):
                        #     self.future_visualize(all_point_list[next_future_ob_index + 8], all_point_list[next_future_ob_index + 9])
                    
                    ## Replan to check for dynamic obstacle
                    waypoints = []

                    # Add the current pose to make sure the path is smooth, get latest pose
                    current_ee_pose_smooth = self.group.get_current_pose().pose
                    waypoints.append(copy.deepcopy(current_ee_pose_smooth))

                    # discard the executed waypoints
                    new_point_list = point_list[executed_waypoint_index+1:success_num]

                    # Add future printing obstacle
                    for k in new_point_list:
                        (current_ee_pose_smooth.position.x, current_ee_pose_smooth.position.y, current_ee_pose_smooth.position.z) = k
                        waypoints.append(copy.deepcopy(current_ee_pose_smooth))

                    (plan2, fraction2) = self.group.compute_cartesian_path(waypoints, 0.01, 0.00, path_constraints = constraints)

                    if fraction2 < 1:
                        ## new obstacle appear
                        # print 'executed latest index', executed_waypoint_index
                        # print 'fraction value', fraction,'\n'
                        print 'new obstacle appeared along the planned path'
                        self.group.stop()
                        executing_state = 0
                        break

                if self.waypoint_execution_status == 3:
                    # waypoint successfully printed
                    # self.print_list_visualize(point_list[:success_num])
                    # print 'status 3', point_list[:success_num]
                    del(point_list[0:success_num-1])
                    finsih_num += success_num

                elif self.waypoint_execution_status == 2 or 4:
                    # state 2 = preempted, state 4 = aborted.
                    # only printed partial waypoint
                    # self.print_list_visualize(point_list[:executed_waypoint_index+1])
                    # print 'status 4', point_list[:executed_waypoint_index+1]
                    del(point_list[:executed_waypoint_index+1]) # delete up till whatever is executed
                    finsih_num += executed_waypoint_index + 1
            self.msg_print.data = False


        self.group.set_path_constraints(None)
        print 'all finish'


