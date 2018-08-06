#!/usr/bin/env python  
import sys
import rospy
import moveit_commander
from cw3_helper.srv import ChangeCollisionObject
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi
from math import cos, sin
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive, Plane, Mesh, MeshTriangle
import geometry_msgs.msg
from geometry_msgs.msg import * 
import moveit_python
from obstacle_command import Scene_obstacle
from control_command import arm_base_control
import re

class enclosure_demo:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('enclosure_demo')

        control = arm_base_control()

        sce = moveit_python.PlanningSceneInterface('odom')
        sce.clear()

        rospy.sleep(0.5)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "odom"
        box_pose.pose.position.x = -0.5
        box_pose.pose.position.y = 2.5
        box_pose.pose.position.z = 0.2
        box_one = "box_1"
        control.scene.add_box(box_one, box_pose, size=(0.5, 0.5, 0.5))
        rospy.sleep(0.5)

        # scene.scene.remove_world_object()
        rospy.sleep(1)

        point_list = []
        point_list += control.straight_line_sample((-1,1), (2,1))
        point_list += control.straight_line_sample((2,1), (2,3))
        point_list += control.straight_line_sample((1,3), (0,3))
        point_list_0 = control.straight_line_sample((0,3), (0,1))

        control.group.set_position_target([1, 2, 0.1], control.group.get_end_effector_link())

        control.group.go(wait = True)


        control.print_list_visualize(point_list)
        control.print_list_visualize(point_list_0, name = 'future_ob')

        
        while not rospy.is_shutdown():
            # Check for enclosure
            control.base_group.set_position_target([0, 0, 0.05], control.base_group.get_end_effector_link())
            result = control.base_group.plan()
            control.base_group.clear_pose_targets()
            
            if len(result.joint_trajectory.points) == 0: 
                print('Check enclosure failed')

                # Remove future obstacle
                current_future_ob_list = sce.getKnownCollisionObjects()

                for point_name in current_future_ob_list:
                    result = re.match('future_ob', point_name)
                    if result: 
                        sce.removeCollisionObject(point_name)
                rospy.sleep(0.05)

                
                random_pose = control.group.get_current_pose(control.group.get_end_effector_link())

                (roll, pitch, yaw) = euler_from_quaternion([random_pose.pose.orientation.x,
                                                            random_pose.pose.orientation.y,
                                                            random_pose.pose.orientation.z,
                                                            random_pose.pose.orientation.w])

                [random_pose.pose.orientation.x, \
                 random_pose.pose.orientation.y, \
                 random_pose.pose.orientation.z, \
                 random_pose.pose.orientation.w] = quaternion_from_euler(roll, pitch, yaw - pi/2)

                (random_pose.pose.position.x, random_pose.pose.position.y, random_pose.pose.position.z) = (0, 2, 0.1)
                control.group.set_pose_target(random_pose)
                control.group.go(wait = True)

                control.group.clear_pose_targets()
                # control.group.set_position_target([0, 2, 0.1], control.group.get_end_effector_link())

                # control.group.go(wait = True)
                control.print_list_visualize(point_list_0, name = 'future_ob')
                rospy.sleep(0.05)

            else: 
                print('Check enclosure successful')
                break





if __name__ == '__main__':
    try:
        enclosure_demo()
        moveit_commander.roscpp_shutdown()
    except rospy.ROSInterruptException:
        pass