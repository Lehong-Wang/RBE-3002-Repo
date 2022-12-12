#!/usr/bin/env python3

import math
import warnings

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path

from constants import *
from utils import *

# Global instance
robot = None
robot_start_timer = None


# robot instance checker decorator
def check_robot(callback_func):
    def wrapper():
        if robot is None:
            raise ValueError(f"{callback_func.func_name}: Robot not initialized when calling \
                             back to {callback_func.func_name}")
        callback_func()

    return wrapper()


# LINEAR_MAX = 0.2
# ANGULAR_MAX = 2
# endregion

class Robot:
    """
    Logical robot class to control logical robot operations
    """
    pose = RobotPose(0, 0, 0)

    # region Initialization
    def __init__(self):
        """
        Robot class constructor
        """
        Utils.pretty_print("Robot Init Started")

        # Initialize node and rename it
        rospy.init_node('lab4_drive_node')

        # Initialize robot pose
        self.set_robot_pose(0, 0, 0)

        # Wheels
        self.wheel = Twist()  # Logical wheel
        self.pub_wheel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_wheel_delay =rospy.Rate(DEFAULT_PUB_RATE)  # 10hz

        # Odometries
        self.sub_odom = rospy.Subscriber('/odom', Odometry, Callbacks.update_odometry)

        # Pose
        self.sub_goal_pose = rospy.Subscriber('/move_base_simple/goal', PoseStamped, Callbacks.call_astar)

        # Timer object, get second with .secs , nanoseconds with .nsecs
        robot_start_timer = rospy.get_rostime()

        Utils.pretty_print("Robot Init Finished")

    def run(self):
        """
        Method to keep connection alive
        :return:
        """
        print("Sleep")
        rospy.sleep(1)
        print("Wake up")

        # self.rotate(math.pi/2, 0.3)
        rospy.spin()

    # endregion

    # region Getters & Setters
    def get_robot_pose(self):
        return self.pose

    def set_robot_pose(self, x, y, theta):
        self.pose.pos.x = x
        self.pose.pos.y = y
        self.pose.theta = theta

    # Feel free to search and replace get_x all over the code back to px, or similar, if you want
    # Also, I shortened originally get_pos_x to get_x for easier usage 
    def get_x(self):
        return self.get_x()

    def set_x(self, x):
        self.pose.pos.x = x

    def get_y(self):
        return self.pose.pos.y

    def set_y(self, y):
        self.pose.pos.y = y

    def get_theta(self):
        return self.pose.theta

    def set_theta(self, theta):
        self.pose.theta = theta

    # endregion

    # region Movement
    def change_wheel_speed(self, linear_speed, angular_speed):
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
        if linear_speed > WARN_WHEEL_LINEAR_SPEED_MAX:
            warn_str = f"Linear Speed above 0.5 m/s is not recommended.\nCurrent: {linear_speed}"
            warnings.warn(warn_str)

        # Length between wheel is 160 mm
        # 0.4 linear speed -> 5 rad/s
        # at 10 rad/s, some off set, roatation sometimes not stable
        # at 20 rad/s, uncontrolable
        if angular_speed > WARN_WHEEL_ANGULAR_SPEED_MAX:
            warn_str = f"Angular Speed above 10 rad/s is not recommended.\nCurrent: {angular_speed}"
            warnings.warn(warn_str)

        # Linear velocity
        self.wheel.linear.x = linear_speed
        self.wheel.linear.y = 0.0
        self.wheel.linear.z = 0.0

        # Angular velocity
        self.wheel.angular.x = 0.0
        self.wheel.angular.y = 0.0
        self.wheel.angular.z = angular_speed

        ### Publish the message
        self.pub_wheel.publish(self.wheel)

    def drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        # Save the initial pose
        initial_px = self.get_x()
        initial_py = self.get_y()

        # Check if reached desired distance
        remaining = float('inf')  # Initialize remaining distance to go to be infinite

        # When remaining distance is larger than LINEAR_DRIVE_THRESHOLD constant
        while remaining > LINEAR_DRIVE_THRESHOLD:
            # Calculate latest remaining distance
            remaining = distance - math.sqrt((self.get_x() - initial_px) ** 2 + (self.get_y() - initial_py) ** 2)

            # print(f"Px: {self.get_x()}, py: {self.get_y()}, init: {(initial_px, initial_py)}")
            # print(f"remaining: {remaining}")
            self.change_wheel_speed(linear_speed, 0)  # Change speed

            rospy.sleep(WHEEL_PUB_DELAY)  # Delay wheel speed change updates

        # Stop the robot when remaining distance to go is smaller than tolerance
        self.change_wheel_speed(0, 0)

        # Print the results
        print(f"Finished Drive {distance}")
        print(f"From: {(initial_px, initial_py)}\tTo: {(self.get_x(), self.get_y())}")

    def rotate(self, angle, angular_speed):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        # Save the initial pose
        initial_pth = self.get_theta()  # Initial theta
        mapped_init_pth = initial_pth + math.pi

        # Process input
        angular_speed = abs(angular_speed)  # Speed is scalar
        angle = (angle + math.pi) % (2 * math.pi) - math.pi  # make angle (-pi, pi)

        # Go reverse when angle < 0
        if angle < 0:
            angular_speed = -angular_speed
        # print(f"@rotate\t Rotate {angle}")

        # Create variables
        target_pth = (mapped_init_pth + angle) % (2 * math.pi) - math.pi
        remaining = float('inf')

        # If When remaining angular update is larger than ANGULAR_ROTATE_THRESHOLD constant
        while remaining > ANGULAR_ROTATE_THRESHOLD:
            remaining = abs(self.get_theta() - target_pth)
            # print(f"update_odometry {(round(self.get_x(),3), round(self.get_y(),3), round(self.get_theta(),3))}")

            # print(f"pth: {self.get_theta()}, init: {(initial_pth)}, target: {target_pth}")
            # print(f"remaining: {remaining}")
            self.change_wheel_speed(0, angular_speed)
            rospy.sleep(WHEEL_PUB_DELAY)

        # Stop the robot
        self.change_wheel_speed(0, 0)
        print(f"Finished Rotate {angle}")
        print(f"From: {initial_pth}\tTo: {self.get_theta()}")

    def go_to(self, target_pose):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param target_pose [PoseStamped] The target pose.
        """
        # Save the initial pose
        initial_pth = self.get_theta()
        initial_px = self.get_x()
        initial_py = self.get_y()

        # Store the message position
        desired_px = target_pose.pose.position.x
        desired_py = target_pose.pose.position.y
        quat_orig = target_pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        desired_pth = yaw

        rospy.loginfo(f"Go_To:\nfrom {(round(self.get_x(), 3), round(self.get_y(), 3), round(self.get_theta(), 3))} \
            \nTo {(round(desired_px, 3), round(desired_py, 3))}")

        wave_point_list = [(desired_px, desired_py)]
        self.execute_wave_point_list(wave_point_list, LINEAR_MAX, ANGULAR_MAX)

        print(f"Final {(round(self.get_x(), 3), round(self.get_y(), 3), round(self.get_theta(), 3))}")
        print(f"Finished\nInitial{(initial_px, initial_py, initial_pth)}")
        print(f"Goal {(round(desired_px, 3), round(desired_py, 3))}")

    def go_to_with_correction(self, pos : Position, linear_speed):
        """
        TODO
        :param dest:
        :param linear_speed:
        :return:
        """
        self.execute_wave_point_list_pid([(pos.x, pos.y)], linear_speed)

    def drive_path_msg(self, msg):
        """
        given a nav_msgs/Path message
        extract wave points
        walk along wave points
        """

        # Saved your code in case you want to revert
        # wave_point_list = []
        # pose_msg_list = msg.plan.poses
        # for pose_msg in pose_msg_list:
        #     x = pose_msg.pose.position.x
        #     y = pose_msg.pose.position.y
        #     wave_point_list.append((x,y))
        #
        # print(f"Robot::drive_path_msg(): Wave Points: {wave_point_list}")
        # # self.run_wave_point_list_pid(wave_point_list, 0.1)
        # self.run_wave_point_list_goto(wave_point_list[1:], LINEAR_MAX, ANGULAR_MAX)

        # TODO: Add checks to msg here? Or is it a calling convention that it will return (x, y) list?
        self.execute_wave_point_list([(pos[0], pos[1]) for pos in msg], 2, 2)

    def execute_wave_point_list(self, wave_point_list, linear_speed, angular_speed):
        for point in wave_point_list:
            self.__goto_wave_point(point, linear_speed, angular_speed)

    def execute_wave_point_list_pid(self, wave_point_list, speed):
        """
        run wave points with pid_control
        Work very well
        turn fast first and move straight
        """

        rospy.loginfo(f"Wave Points: {wave_point_list}\nSpeed: {speed}")

        for i, point in enumerate(wave_point_list):
            total_dist = math.sqrt((self.get_x() - point[0]) ** 2 + (self.get_y() - point[1]) ** 2)
            expected_time = total_dist / speed
            print(f"Expected time: {expected_time}")
            start_t = Utils.get_time(rospy)

            # distance to destination, for checking if goal is passed
            dist_list = []
            dist = math.sqrt((self.get_x() - point[0]) ** 2 + (self.get_y() - point[1]) ** 2)
            dist_list.append(dist)

            angular_speed_list = [0.0, 0.0, 0.0]
            # angular speed for d controller
            d_angle = 0
            # set a virtrual goal behind actrual goal to pervent angle fluctruation near goal
            virtrual_goal = Position(point[0] + 0.5 * (point[0] - self.get_x()),
                                     point[1] + 0.5 * (point[1] - self.get_y()))

            delay = rospy.Rate(CHECK_POS_RATE_PID)
            while dist > CHECK_POS_THRESHOLD:
                current_t = Utils.get_time(rospy)
                elapsed_t = current_t - start_t
                dist = math.sqrt((self.get_x() - point[0]) ** 2 + (self.get_y() - point[1]) ** 2)
                dist_list.append(dist)
                # print(dist)
                # if not reached after perdicted time
                if elapsed_t > TIME_TOLERANCE_FACTOR * expected_time:
                    # rospy.loginfo(f"Wave point {point} not reached in time, recalculating route.")
                    # if moving away from target, stop
                    if dist_list[-1] > dist_list[-2] and dist_list[-2] > dist_list[-3]:
                        break

                linear_speed, angular_speed = Utils.PID.get_pid_speed(self.get_robot_pose(),
                                                                      virtrual_goal, speed, d_angle)
                # smooth start and stop
                if i == 0:
                    linear_speed, angular_speed = Utils.PID.smooth_start(linear_speed, angular_speed, dist_list)
                if i == len(wave_point_list) - 1:
                    linear_speed, angular_speed = Utils.PID.smooth_stop(linear_speed, angular_speed, dist_list)
                self.change_wheel_speed(linear_speed, angular_speed)
                # calculate derivative of angular_speed
                angular_speed_list.append(angular_speed)
                d_angle = 1 / 2 * (angular_speed_list[-1] + angular_speed_list[-2])

                # delay.sleep()

            self.change_wheel_speed(0, 0)
            print(f"Reached {point}")
            # rospy.sleep(1)

        rospy.loginfo(f"Goal {wave_point_list[-1]} Reached !")

    def __goto_wave_point(self, point, linear_speed, angular_speed):
        """
        Private helper function to go to one wave point
        :param point:
        :param linear_speed:
        :param angular_speed:
        :return:
        """
        desired_px, desired_py = point
        initial_px, initial_py = self.get_x(), self.get_y()

        print(f"Going to Wave Point {(round(desired_px, 3), round(desired_py, 3))}")
        print(f"Current pose {(round(self.get_x(), 3), round(self.get_y(), 3), round(self.get_theta(), 3))}")

        # Calculate the rotation needed based on desired position
        delta_x = desired_px - initial_px
        delta_y = desired_py - initial_py
        angle = math.atan2(delta_y, delta_x)

        # Call rotate()
        # angle_diff = Utils.merge_angle("sub", angle, self.get_theta())
        angle_diff = Angle(angle) - Angle(self.get_theta())
        print(f"Rotating {angle_diff}")
        self.rotate(angle_diff, angular_speed)
        self.change_wheel_speed(0, 0)
        rospy.sleep(0.5)
        print(f"Current pose {(round(self.get_x(), 3), round(self.get_y(), 3), round(self.get_theta(), 3))}")

        self.go_to_with_correction(Position(desired_px, desired_py), linear_speed)

        self.change_wheel_speed(0, 0)
        rospy.sleep(0.5)

    # endregion


# region Callbacks
class Callbacks:

    def __init__(self):
        raise NotImplementedError("Callback class should never be instantiated")

    @staticmethod
    @check_robot
    def update_odometry(msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """

        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        new_theta = yaw

        robot.set_robot_pose(new_x, new_y, new_theta)
        # print(f"update_odometry {(round(robot.get_x(),3), round(robot.get_y(),3), \
        #   round(robot.get_theta(),3))}")

    # TODO: Since you called it a callback I put it here.
    #  However, I think it should go under utils probably
    @staticmethod
    @check_robot
    def pos_to_robot_frame_callback(msg):
        """

        :param msg:
        :return:
        """

        x2 = msg.pose.position.x
        y2 = msg.pose.position.y
        quat_orig = msg.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat_list)
        th2 = yaw

        robot_pose = robot.get_robot_pose()
        target_pos = Position(x2, y2)

        Utils.world_to_robot_frame(robot_pose, target_pos, th2)

    @staticmethod
    @check_robot
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
        initial_pose.pose.position.x = self.get_x()
        initial_pose.pose.position.y = self.get_y()
        initial_pose.pose.orientation.w = self.get_theta()

        path = path_plan(initial_pose, msg, A_STAR_PATH_TOLERANCE)
        rospy.loginfo("sent path from rviz")

        # print(send)
        robot.drive_path_msg(path)

    @staticmethod
    @check_robot
    def pid_callback(msg):
        # Store the message position

        desired_px = msg.pose.position.x
        desired_py = msg.pose.position.y

        print(f"PoseStamped {(round(desired_px, 3), round(desired_py, 3))}")
        print(f"Current pose {(round(robot.get_x(), 3), round(robot.get_y(), 3), round(robot.get_theta(), 3))}")

        robot.execute_wave_point_list_pid([(desired_px, desired_py)], LINEAR_MAX)


# endregion


if __name__ == '__main__':
    robot = Robot()
    robot.run()
