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
from math import pi
from math import cos
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import datetime
from numpy import NaN
import pandas as pd
import os
from std_msgs.msg import String


class save_data:
    def __init__(self):
    # Initialize moveit_commander and rospy node 
        rospy.init_node('plot')

        self.joint_callback = rospy.Subscriber("/joint_states", JointState, self.joint_callback)

        # self.callback = rospy.Subscriber("/joint_states", JointState, self.callback)

        experiment = rospy.Publisher('experiment_name',
                                     String,
                                     queue_size=20)
        rospy.Subscriber("/experiment_name", String, self.experiment_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.base_position_x = []
        self.base_position_y = []
        self.base_position_z = []
        self.eef_position_x = []
        self.eef_position_y = []
        self.eef_position_z = []
        self.position_time = []

        self.js_0 = []
        self.js_1 = []
        self.js_2 = []
        self.js_time = []

        self.name = None
        self.startime = datetime.datetime.now()
        rospy.spin()

    def re_initial(self):
        self.base_position_x = []
        self.base_position_y = []
        self.base_position_z = []
        self.eef_position_x = []
        self.eef_position_y = []
        self.eef_position_z = []
        self.position_time = []

        self.js_0 = []
        self.js_1 = []
        self.js_2 = []
        self.js_time = []
        self.startime = datetime.datetime.now()

    def save_date(self):

        rospy.sleep(0.1)

        plt2 = plt.figure(self.name + "EEf path", figsize=(6, 6))
        plt.plot(self.eef_position_x ,self.eef_position_y, '-bo',linewidth = 1, markersize = 3)
        plt.plot(self.base_position_x ,self.base_position_y, '-yo',linewidth = 1, markersize = 3)
        plt.plot(self.eef_position_x[0] ,self.eef_position_y[0], 'ro', markersize = 8)
        plt.plot(self.eef_position_x[-1] ,self.eef_position_y[-1], 'go', markersize = 8)
        x_min = min(min(self.eef_position_x), min(self.base_position_x))
        x_max = max(max(self.eef_position_x), max(self.base_position_x))
        y_min = min(min(self.eef_position_y), min(self.base_position_y))
        y_max = max(max(self.eef_position_y), max(self.base_position_y))
        plt.xlim(x_min - 1, x_max + 1)
        plt.ylim(y_min - 1, y_max + 1)
        plt.legend(['EEF Trajactory', 'Base Trajactory', 'Starting point', 'Ending point'], fontsize=12, bbox_to_anchor=(1.0, 1))
        plt.title('Path')
        plt.xlabel("X axis (m)")
        plt.ylabel("y axis (m)")

        # fun_sub = lambda a, b: a - b
        # speed_j0 = [0] + list(map(fun_sub, self.js_0[1:], self.js_0[0:-1]))
        # speed_j1 = [0] + list(map(fun_sub, self.js_1[1:], self.js_1[0:-1]))
        # speed_j2 = [0] + list(map(fun_sub, self.js_2[1:], self.js_2[0:-1]))

        # plt3 = plt.figure(self.name + "Base joint state", figsize=(10, 6))
        # plt.plot(self.js_time[0:len(speed_j0)], speed_j0, 'b', linewidth = 0.5)
        # plt.plot(self.js_time[0:len(speed_j1)], speed_j1, 'r', linewidth = 0.5)
        # plt.plot(self.js_time[0:len(speed_j2)], speed_j2, 'g', linewidth = 0.5)
        # plt.xlim(0, max(self.js_time) + 1)
        # plt.legend(['Joint 0', 'Joint 1', 'Joint 2'], fontsize=8, bbox_to_anchor=(1.0, 1))
        # plt.title('Base Joint speed')
        # plt.xlabel("Time from start (sec)")
        # plt.ylabel("Joint_state speed (m/s)")

        # plt.show()

        # plt1.savefig(self.name + 'base.png', dpi=plt1.dpi)
        plt2.savefig('experiment_data/' + str(self.name) + '/' + self.name +'_path.png', dpi=plt2.dpi)
        # plt3.savefig('experiment_data/' + str(self.name) + '/' + self.name + '_joint_state.png', dpi=plt3.dpi)


        list_position=[self.position_time,
                        self.base_position_x,
                        self.base_position_y,
                        self.base_position_z,
                        self.eef_position_x,
                        self.eef_position_y,
                        self.eef_position_z]

        position_data = pd.DataFrame(data=list_position)
        position_data.to_csv('experiment_data/' + str(self.name) + '/' + self.name +'_position.csv',encoding='gbk')

        # list_cmd_v=[self.js_time[0:len(speed_j0)], speed_j0, speed_j1, speed_j2]

        # cmd_v_data = pd.DataFrame(data=list_cmd_v)
        # cmd_v_data.to_csv('experiment_data/' + str(self.name) + '/' + self.name +'_cmd_v.csv',encoding='gbk')

        rospy.sleep(0.1)

    def joint_callback(self, data):

        rospy.sleep(0.3)
        try:
            base_record = self.tf_buffer.lookup_transform('odom','base_link', rospy.Time.now(), rospy.Duration(0.1))
            eef_record = self.tf_buffer.lookup_transform('odom','arm_link_ee', rospy.Time.now(), rospy.Duration(0.1))

            # print round(eef_record.transform.translation.x, 4), round(eef_record.transform.translation.y, 4)
            self.base_position_x.append(round(base_record.transform.translation.x, 4))
            self.base_position_y.append(round(base_record.transform.translation.y, 4))
            self.base_position_z.append(round(base_record.transform.translation.z, 4))
            self.eef_position_x.append(round(eef_record.transform.translation.x, 4))
            self.eef_position_y.append(round(eef_record.transform.translation.y, 4))
            self.eef_position_z.append(round(eef_record.transform.translation.z, 4))
            self.position_time .append(rospy.Time.now())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(0.1)

    # def callback(self, data):

    #     finshtime = datetime.datetime.now()

    #     self.js_0.append(data.position[0])
    #     self.js_1.append(data.position[1])
    #     self.js_2.append(data.position[2])
    #     self.js_time.append((finshtime-self.startime).seconds)

    def experiment_callback(self, data):
        print(data)
        if data:
            if data.data == 'Finish': 

                self.save_date()
                print('Save data:', self.name, 'Successful')

 
            else:
                self.re_initial()
                self.name = data.data
                self.mkdir('experiment_data/' + str(self.name))

    def mkdir(self, path):
     
        folder = os.path.exists(path)
     
        if not folder:                  
            os.makedirs(path)          
            print "---  new folder...  ---"
            print "---  OK  ---"
     
        else:
            print "---  There is this folder!  ---"

if __name__ == '__main__':

    
    try:
        save_data()
    except rospy.ROSInterruptException:
        pass