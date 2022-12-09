#!/usr/bin/env python3

import math
import warnings
import copy

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path


from path_planner import *


class Mapping:

    def __init__(self):
        rospy.init_node("lab4_mapping_node")


        self.sub_cspace_map = rospy.Subscriber('/path_planner/cspace_map', OccupancyGrid, PathPlanner.print_map)








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




    def calc_frontior(self, mapdata):
        width = mapdata.info.width
        height = mapdata.info.height
        map_array = mapdata.data



    @staticmethod
    def find_initial_frontior(mapdata):

        # list of (x,y) for frontier
        frontier_list = []
        for i,value in enumerate(mapdata.data):
            if value != 0:
                continue
            x,y = PathPlanner.index_to_grid(mapdata,i) 
            if Mapping.neighbor_have_unknown(mapdata, x, y):
                frontier_list.append((x,y))

        print(f"Initial Frontier List: {frontier_list}")
        return frontier_list


    @staticmethod
    def neighbor_have_unknown(mapdata, x, y):

        neib_coords = PathPlanner.neighbors_of_8(mapdata, x, y)

        for coord in neib_coords:
            value = PathPlanner.grid_to_index(mapdata, coord[0], coord[1])
            if value == -1:
                return True

        return False


    @staticmethod
    def inflate(mapdata, frontier_list):
        for coord in frontier_list:
            pass

















    def run(self):

        rospy.spin()



if __name__ == '__main__':
    Mapping().run()





