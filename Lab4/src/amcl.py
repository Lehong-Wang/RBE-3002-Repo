#!/usr/bin/env python3

import math
import warnings
import copy
import rospy
from nav_msgs.srv import GetPlan, GetMap
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import GridCells, OccupancyGrid, Path, MapMetaData
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray
from sensor_msgs.msg import LaserScan, PointCloud, PointCloud2
import priority_queue
from std_srvs.srv import Empty
import rosservice

import drive
import path_planner

class AMCL_Test:


    
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node("amcl_test")

        ## Create subscribers
        # self.laser_scan_sub = rospy.Subscriber('/scan', LaserScan, self.testing)
        # self.tf_sub = rospy.Subscriber('/tf', tfMessage, queue_size=10)
        # self.initialpose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.update_initialpose)
        # self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.update_map)

        # self.goal_pose_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.call_astar)
        # ## Create publishers
        # self.amcl_pose_pub = rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped, queue_size=10)
        # self.particlecloud_pub = rospy.Publisher('/particlecloud', PoseArray, queue_size=10)
        # self.tf_pub = rospy.Publisher('/tf', tfMessage, queue_size=10)
        # self.map_metadata = rospy.Publisher('/map_metadata', MapMetaData, queue_size=10)
        # self.map = rospy.Pulisher('/map', OccupancyGrid, queye_size=10)

        self.amcl_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.error_check)
        
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        
        # Robot pos
        self.px = 0
        self.py = 0
        self.pth = 0

        # Laser Scan
        self.laser_scan = LaserScan()

        # Create variables based on previously created files
        self.dr = drive.Lab2()
        self.path_plan = path_planner.PathPlanner()
        
        print("finished init")

    # def testing(self):
    #     map = self.path_plan.request_map()
    #     pad_map = self.path_plan.calc_cspace(map, 3)

        

    def error_check(self, msg):
        error_margin = 0.1
        error_x = abs(self.px - msg.pose.pose.position.x)
        error_y = abs(self.py - msg.pose.pose.position.y)
        if (error_x < error_margin and error_y < error_margin):
            print("Robot is within the point cloud estimate")
            # Publish to initialpose topic
            self.dr.rotate(0,0)
            self.initialpose_pub.publish(msg)
        else:
            self.dr.rotate(360,0.2)


    def update_initialpose(self, msg):
        """
        Updates the initial pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [PoseWithCovarianceStamped] The current pose information.
        """
        self.px = msg.pose.position.x
        self.py = msg.pose.position.y
        quat_orig = msg.pose.orientation
        quat_list = [ quat_orig.x , quat_orig.y , quat_orig.z , quat_orig.w]
        ( roll , pitch , yaw ) = euler_from_quaternion ( quat_list )
        self.pth = yaw


# ----------------------------------- Main --------------------------
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.loginfo("Global Localization Service Call")
        rospy.wait_for_service('global_localization')
        rospy.ServiceProxy('global_localization', Empty)
        #rosservice.call_service('/global_localization', Empty)
        print("AMCL Sleep")
        rospy.sleep(1)
        rospy.spin()
        print("AMCL Wake up")
        

if __name__ == '__main__':
    AMCL_Test().run()
