#!/usr/bin/env python3



import math
import warnings
import copy
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped



class Header:


    def __init__(self):
        rospy.init_node("lab4_head_node")
        pass





