#!/usr/bin/env python3

import math
import warnings
import copy
import rospy
from nav_msgs.srv import GetPlan, GetMap
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import GridCells, OccupancyGrid, Path, MapMetaData
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray, tfMessage
from sensor_msgs import LaserScan
import priority_queue
from std_srvs import GlobalLocalization

class AMCL_Test:


    
    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node("amcl_test")

        ## Create subscribers
        self.laser_scan_sub = rospy.Subscriber('/scan', LaserScan, self.get_laser_scan)
        self.tf_sub = rospy.Subscriber('/tf', tfMessage, queue_size=10)
        self.initialpose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.update_initialpose)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.update_map)

        self.goal_pose_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.call_astar)
        ## Create publishers
        self.amcl_pose_pub = rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped, queue_size=10)
        self.particlecloud_pub = rospy.Publisher('/particlecloud', PoseArray, queue_size=10)
        self.tf_pub = rospy.Publisher('/tf', tfMessage, queue_size=10)
        self.map_metadata = rospy.Publisher('/map_metadata', MapMetaData, queue_size=10)
        self.map = rospy.Pulisher('/map', OccupancyGrid, queye_size=10)

        # Create global localization service
        global_loc_srv = rospy.Service('global_loc', GlobalLocalization)

        # Robot pos
        self.px = 0
        self.py = 0
        self.pth = 0

        # Laser Scan
        self.laser_scan = LaserScan()

        print("finished init")


    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()

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

     def call_astar(self, msg):
        """
        Creates the path using Astar from Rviz
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The goal pose.
        """
        ## Create service proxy
        path_plan = rospy.ServiceProxy('plan_path', GetPlan)
        initial_pose = PoseStamped()
        
        # Save the initial pose
        initial_pose.pose.position.x = self.px
        initial_pose.pose.position.y = self.py
        initial_pose.pose.orientation.w = self.pth
        
        tolerance = 0.1
        
        send = path_plan(initial_pose, msg, tolerance)
        rospy.loginfo("sent path from rviz")

        # print(send)
        #self.drive_along_path(send)
    
    def get_laser_scan(self, scanMsg):
        self.laser_scan.header = scanMsg.header
        self.laser_scan.angle_min = scanMsg.angle_min
        self.laser_scan.angle_max = scanMsg.angle_max
        self.laser_scan.angle_increment = scanMsg.angle_increment
        self.laser_scan.time_increment = scanMsg.time_increment
        self.laser_scan.scan_time = scanMsg.scan_time
        self.laser_scan.range_min = scanMsg.range_min
        self.laser_scan.range_max = scanMsg.range_max
        self.laser_scan.ranges = scanMsg.ranges
        self.laser_scan.intensities = scanMsg.intensities

    

    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
        None in case of error.
        """
        rospy.loginfo("Requesting the map")
        try:
            rospy.wait_for_service('static_map') # Block until service is available
            grid = rospy.ServiceProxy('static_map', GetMap)
        except Exception:
            print(f"Error when requesting map\n{Exception}")
            return None
    return grid().map
        
if __name__ == '__main__':
    AMCL_Test().run()
