#!/usr/bin/env python  
import sys
import rospy
import moveit_commander
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, atan2, cos, sin, tan, sqrt
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
import geometry_msgs.msg
from geometry_msgs.msg import * 
import moveit_python
import numpy as np

class Scene_obstacle(object):

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('Scene_Obstacle')

        """ Create a planning scene interface; it uses both C++ wrapped methods and scene manipulation topics. """
        self.scene = moveit_commander.PlanningSceneInterface()
        self.pub_co = rospy.Publisher('collision_object', CollisionObject, queue_size=100)

        # initial printing number 
        self._printing_number = 0
        self.further_printing_number = 0

    # Demo 1 scene
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
        box_name = name + str(self._printing_number)
        self.pub_co.publish(self.make_box(box_name, box_pose, size=(length, 0.01, 0.01)))

        rospy.sleep(0.05)

    def print_list_visualize(self, way_points):

        if len(way_points)>0:
            way_points = [way_points[0]] + way_points
            differential = map(self.printing_visualize, way_points[:-1], way_points[1:])

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


        self.further_printing_number += 1
        if self.further_printing_number == 3: self.further_printing_number = 0
        box_name = 'future_point' + str(self.further_printing_number)
        self.pub_co.publish(self.make_box(box_name, box_pose, size=(length, 0.01, 0.01)))

        rospy.sleep(0.05)

    def print_future_visualize(self, way_points, index, step = 5, length = 3, status = True, point_num = 1):

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
                # Remove obstacle
                else:
                    loop_num = step + length + index - len(point_list) -1 
                    for i in range(loop_num):
                        self.future_visualize(way_points[-1], way_points[-1])

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

