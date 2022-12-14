#!/usr/bin/env python3

import math
import warnings
import copy
import rospy
from nav_msgs.srv import GetPlan, GetMap
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import GridCells, OccupancyGrid, Path, MapMetaData
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped, PoseArray
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
        
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.msg_cmd_vel = Twist()
        self.rate = rospy.Rate(50)

        # Robot pos
        self.px = 0
        self.py = 0
        self.pth = 0

        # Laser Scan
        self.laser_scan = LaserScan()
        
        print("finished init")

        
    def error_check(self, msg):
        error_margin = .2
        
        error_x = abs(self.px - msg.pose.pose.position.x)
        error_y = abs(self.py - msg.pose.pose.position.y)
        if (error_x < error_margin and error_y < error_margin):
            print("Robot is within the point cloud estimate")
            # Publish to initialpose topic
            self.rotate(0,0)
            self.initialpose_pub.publish(msg)
        else:
            self.rotate(360,0.5)
        print("Error_X: "+ error_x)
        print("Error_Y: "+ error_y)

    def send_speed(self, linear_speed, angular_speed):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """

        # at most any speed, completely off track by 5 m
        # at 0.5 m/s, off by 3m
        # at 0.6 m/s, off by 1m
        # at 0.7 m/s, uncontrolable
        # recomand speed <= 0.4 m/s
        if linear_speed > 0.5:
            warn_str = f"Linear Speed above 0.5 m/s is not recommended.\nCurrent: {linear_speed}"
            warnings.warn(warn_str)
        # Length between wheel is 160 mm
        # 0.4 linear speed -> 5 rad/s
        # at 10 rad/s, some off set, roatation sometimes not stable
        # at 20 rad/s, uncontrolable
        if angular_speed > 10:
            warn_str = f"Angular Speed above 10 rad/s is not recommended.\nCurrent: {angular_speed}"
            warnings.warn(warn_str)

        # Linear velocity
        self.msg_cmd_vel.linear.x = linear_speed
        self.msg_cmd_vel.linear.y = 0.0
        self.msg_cmd_vel.linear.z = 0.0

        # Angular velocity
        self.msg_cmd_vel.angular.x = 0.0
        self.msg_cmd_vel.angular.y = 0.0
        self.msg_cmd_vel.angular.z = angular_speed

        ### Publish the message
        self.pub.publish(self.msg_cmd_vel)
        # print(self.msg_cmd_vel)
        self.rate.sleep()

    def rotate(self, angle, aspeed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        # Save the initial pose
        initial_pth = self.pth
        mapped_init_pth = initial_pth + math.pi
        
        # Process input
        aspeed = abs(aspeed)    # speed is positive
        angle = (angle+math.pi) % (2*math.pi) - math.pi   # make angle (-pi, pi)
        # angle = (angle+math.pi) % (2*math.pi)   # make angle (0, 2*pi)
        
        # Go reverse when angle < 0
        if angle < 0:
            aspeed = -aspeed
        # print(f"@rotate\t Rotate {angle}")
        
        # Create variables
        target_pth = (mapped_init_pth+angle) % (2*math.pi) - math.pi
        error = float('inf')
        tolerance = 0.05

        # If reached desired distance
        while (error > tolerance):
            error = abs(self.pth - target_pth)
            # print(f"update_odometry {(round(self.px,3), round(self.py,3), round(self.pth,3))}")

            # print(f"pth: {self.pth}, init: {(initial_pth)}, target: {target_pth}")
            # print(f"error: {error}")
            self.send_speed(0, aspeed)
            rospy.sleep(0.05)


    # def update_initialpose(self, msg):
        # """
        # Updates the initial pose of the robot.
        # This method is a callback bound to a Subscriber.
        # :param msg [PoseWithCovarianceStamped] The current pose information.
        # """
        # self.px = msg.pose.position.x
        # self.py = msg.pose.position.y
        # quat_orig = msg.pose.orientation
        # quat_list = [ quat_orig.x , quat_orig.y , quat_orig.z , quat_orig.w]
        # ( roll , pitch , yaw ) = euler_from_quaternion ( quat_list )
        # self.pth = yaw

    def global_loc(self):
        try:
            global_local = rospy.ServiceProxy('global_localization', Empty)
            response = global_local()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
# ----------------------------------- Main --------------------------
    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        print("AMCL Sleep")
        rospy.sleep(1)

        rospy.loginfo("Global Localization Service Call")
        self.global_loc()
        print("AMCL Wake up")
        rospy.spin()
        

if __name__ == '__main__':
    AMCL_Test().run()
    # AMCL_Test().global_loc()
