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
from std_srvs.srv import Empty as srv_Empty
from std_msgs.msg import Int32, Empty
import rosservice

import drive
import path_planner

class AMCL_Test:



    def __init__(self):
        """
        Class constructor
        """
        rospy.init_node("amcl_test")


        self.amcl_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_pos)

        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.init_phase_pub = rospy.Publisher('/task_control/init_phase', Int32, queue_size=10)
        self.request_init_phase_sub = rospy.Subscriber('/task_control/request_init_phase', Empty, self.send_phase)

        # Robot pos
        self.px = 0
        self.py = 0
        self.pth = 0

        self.is_calibrated = False

        # max admitable variance
        self.variance_threshold = 0.005
        self.calib_rot_speed = 0.4

        print("finished init")





    def update_pos(self, msg):
        """/amcl msg, PoseWithCovarianceStamped"""
        covariance = msg.pose.covariance
        var_x = covariance[0]
        var_y = covariance[7]
        var_th = covariance[-1]

        print(f"Pos: {(round(msg.pose.pose.position.x,3), round(msg.pose.pose.position.y,3))}")
        print(f"Var: {(round(var_x,3), round(var_y,3), round(var_th,3))}")

        # print(f"IS calibrated: {self.is_calibrated}")
        if not self.is_calibrated:
            # amcl variance below torlerence
            # var_th is usually larger than others
            if max(var_x, var_y, var_th) < self.variance_threshold:

                self.send_speed(0,0)
                self.is_calibrated = True
                return
            rospy.loginfo(f"AMCL not stable: \nVar: {(var_x, var_y, var_th)}")
            self.send_speed(0, self.calib_rot_speed)

        if self.is_calibrated:
            # update pos with amcl
            self.px = msg.pose.pose.position.x
            self.py = msg.pose.pose.position.y
            quat_orig = msg.pose.pose.orientation
            quat_list = [ quat_orig.x , quat_orig.y , quat_orig.z , quat_orig.w]
            ( roll , pitch , yaw ) = euler_from_quaternion ( quat_list )
            self.pth = yaw
            print(f"update_odometry {(round(self.px,3), round(self.py,3), round(self.pth,3))}")     

            if max(var_x, var_y, var_th) > self.variance_threshold * 2:
                rospy.loginfo(f"AMCL Variance too big:\nVar: {(var_x, var_y, var_th)}")
                rospy.loginfo("Reinitializing AMCL")
                self.is_calibrated = False
                self.global_loc()
                self.send_speed(0, self.calib_rot_speed)



    def send_speed(self, linear_speed, angular_speed):
        msg = Twist()

        msg.linear.x = linear_speed
        msg.angular.z = angular_speed

        self.vel_pub.publish(msg)




    def global_loc(self):
        try:
            rospy.loginfo("Calling Global_Localization Service")
            global_local = rospy.ServiceProxy('global_localization', srv_Empty)
            response = global_local()
            print(response)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)



    def send_phase(self, msg):
        rospy.loginfo("Recieived Phase request, Sending Phase = 3")
        self.init_phase_pub.publish(3)



    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        print(self.is_calibrated)
        print("AMCL Sleep")
        rospy.sleep(1)
        # initilize to phasee 3

        rospy.loginfo("Global Localization Service Call")
        self.global_loc()
        print("AMCL Wake up")
        rospy.spin()


if __name__ == '__main__':
    AMCL_Test().run()
    # AMCL_Test().global_loc()
