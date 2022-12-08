import math
import warnings

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path




class Mapping:

    def __init__(self):
        rospy.init_node("lab4_mapping_node")








    # TESTED
    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        try:
            rospy.wait_for_service('dynamic_map') # Block until service is available
            grid = rospy.ServiceProxy('dynamic_map', GetMap)
        except Exception:
            print(f"Error when requesting map\n{Exception}")
            return None
        return grid().map












