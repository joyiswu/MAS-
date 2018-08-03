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


def make_box(name, pose, size = (0.5, 0.5, 0.5)):
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


if __name__ == '__main__':
    rospy.init_node('youwasp_tf_listener')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    pub_co = rospy.Publisher('collision_object', CollisionObject, queue_size=100)


    while not rospy.is_shutdown():

            try:

                object_pose_record = tf_buffer.lookup_transform('odom', 'base_link', rospy.Time.now(), rospy.Duration(1.0))
                # print(object_pose_record.transform.translation)
                position = object_pose_record.transform.translation
                box_pose = geometry_msgs.msg.PoseStamped()
                box_pose.header.frame_id = "odom"
                box_pose.pose.position.x = position.x - 1
                box_pose.pose.position.y = position.y - 1
                box_pose.pose.position.z = position.z - 1
                pub_co.publish(make_box('box', box_pose))

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(1)
                continue