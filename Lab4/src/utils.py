# OS Import
import warnings
import inspect
from types import ModuleType

# Math Import
import math
from math import sin, cos
import numpy as np

# Local Import
from constants import *

# TODO:
# I defined these two classes for the sake of generalizing apis / avoid further checks
# Fell free to remove them and search & replace things like robot.pose.pos.x back to robot.px
class Position:

    x, y = 0, 0

    def __init__(self, x, y):
        self.x = x
        self.y = y

class RobotPose:
    pos = Position(0, 0)
    theta = 0

    def __init__(self, x, y, theta):
        self.pos.x = x
        self.pos.y = y
        self.theta = theta

class Angle:

    def __init__(self, angle):
        self.angle = angle

    def __add__(self, other):
        try:
            assert isinstance(other, Angle)
        except Exception as e:
            warnings.warn(f"{e}: Add angle operation cannot be applied between"
                          f" {self} and {other}: Non-angles")
        return (self.angle + other.angle + 2 * math.pi) % (2 * math.pi) - math.pi

    def __sub__(self, other):
        try:
            assert isinstance(other, Angle)
        except Exception as e:
            warnings.warn(f"{e}: Sub angle operation cannot be applied between"
                          f" {self} and {other}: Non-angles")
        return (self.angle - other.angle + math.pi) % (2 * math.pi) - math.pi

class Utils:

    def __init__(self):
        raise NotImplementedError("Utility class should never be instantiated")

    @staticmethod
    def pretty_print(msg):
        try:
            msg = str(msg)
            if isinstance(msg, str):
                print(msg.center(STR_PRETTIER_LENGTH, STR_PRETTIER_FILLCHAR))
                print("")
        except Exception as e:
            print(e)

    @staticmethod
    def merge_angle(operation, th1, th2):
        """
        do operation (+/-) on two angle in range (-pi, pi)
        """
        if operation == "add":
            ret = (th1 + th2 + 2 * math.pi) % (2 * math.pi) - math.pi
        elif operation == "sub":
            ret = (th1 - th2 + math.pi) % (2 * math.pi) - math.pi
        else:
            warn_str = f"Merge angle operation not recognized. Choose add or sub\n Given {operation}"
            warnings.warn(warn_str)
            return 0
        return ret

    @staticmethod
    def get_time(rospy : ModuleType, now=None):
        """
        Returns the time to 3 digits
        Convert if given ros Timer object
        """
        try:
            assert inspect.ismodule(rospy)
        except Exception as e:
            raise ModuleNotFoundError("Utils:get_time: cannot find rospy")

        if not now:
            now = rospy.get_rostime()
        time = now.secs + int(str(now.nsecs)[:3]) * 0.001
        # print(f"get_time: {time}")
        return time

    @staticmethod
    def world_to_robot_frame(robot_pose : RobotPose,
                             target_pos : Position,
                             th=0):
        """
        Transform world frame to robot frame
        :param robot_pose: Robot current pose, including x, y, and theta
        :param target_pos: Target position, including x and y
        :param th: Target theta (default is 0)
        :return: (xr, yr)
        """
        ROBOT_POSE_ARG_LENGTH = 3
        TARGET_POS_ARG_LENGTH = 2

        x0, y0, th0, x, y = \
            Utils.check_robot_pose_target_position(robot_pose, target_pos)

        trans_mat = np.matrix([[x - x0], [y - y0]])
        rot_mat = np.matrix([[cos(-th0), -sin(-th0)], [sin(-th0), cos(-th0)]])
        robot_frame = np.matmul(rot_mat, trans_mat)
        xr = robot_frame[0, 0]
        yr = robot_frame[1, 0]

        print(f"\nWorld Frame: {(x, y, th)}")
        print(f"Robot: {(x0, y0, th0)}")
        print(trans_mat)
        print(robot_frame)
        print(f"Robot Frame: {(xr, yr)}")
        return xr, yr

    @staticmethod
    def check_robot_pose_target_position(robot_pose : RobotPose,
                                         target_pos : Position):
        """
        Check if the robot pose and target position makes sense
        :param robot_pose: Robot current pose, including x, y, and theta
        :param target_pos: Target position, including x and y
        :return: return robot_x, robot_y, robot_th, target_x, target_y if no errors.
            Else Raise error if found issues with check
        """
        ROBOT_POSE_ARG_LENGTH = 3
        TARGET_POS_ARG_LENGTH = 2

        try:
            assert len(robot_pose) == ROBOT_POSE_ARG_LENGTH
            x0 = robot_pose[0]
            y0 = robot_pose[1]
            th0 = robot_pose[2]

            assert len(target_pos) == TARGET_POS_ARG_LENGTH
            x = target_pos[0]
            y = target_pos[1]
        except:
            raise ValueError("check_robot_pose_and_target_position_args: "
                             "value parsing error")

        return x0, y0, th0, x, y


    class PID:

        def __init__(self):
            raise NotImplementedError("Utility::PID class should never be instantiated")

        @staticmethod
        def get_pid_speed(robot_pose : RobotPose,
                          target_pos : Position,
                          speed, d_angle):
            """
            PID controller
            linear speed is constant, angular speed is PD controled
            :param robot_pose: Robot current pose, including x, y, and theta
            :param target_pos Target  in (x,y)
            :param speed            Linear speed
            :d_angle                caculated derivative of angular speed
            :return                 (linear_speed, angular_speed)
            """

            x1, y1, theta, x2, y2 = \
                Utils.check_robot_pose_target_position(robot_pose, target_pos)

            kp_angle = 6
            kd_angle = 1

            target_angle = math.atan2(y2 - y1, x2 - x1)
            p_angle = Utils.merge_angle("sub", target_angle, theta)
            # print(f"th:{theta}, tar:{target_angle}, p:{p_angle}")

            v_angle = kp_angle * p_angle - kd_angle * d_angle

            if abs(v_angle) > ANGULAR_MAX:
                v_angle *= ANGULAR_MAX / abs(v_angle)

            # robot.change_wheel_speed(speed, v_angle)
            return speed, v_angle

        @staticmethod
        def smooth_start(linear_speed, angular_speed, dist_list):
            """slowly accelerates for first 10 commands"""
            full_start_length = 10
            current_length = len(dist_list)
            if current_length < full_start_length:
                linear_speed = linear_speed * current_length / full_start_length
                # print(f"Speed: {linear_speed}")

            return linear_speed, angular_speed

        @staticmethod
        def smooth_stop(linear_speed, angular_speed, dist_list):
            """slowly decelerates for the last 0.1 m"""
            start_break_dist = 0.1
            current_dist = dist_list[-1]
            if current_dist < start_break_dist:
                linear_speed = linear_speed * current_dist * 10
                # print(f"Speed: {linear_speed}")
            return linear_speed, angular_speed

