#!/usr/bin/env python

import rospkg
import rosbag
import rospy
import math
from numpy import *
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Int32, String


bag = rosbag.Bag('rrr.bag')


# Store position velocity and time information
for msg in bag.read_messages():
	print(msg)
sadada
