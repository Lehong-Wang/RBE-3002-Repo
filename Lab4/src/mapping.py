#!/usr/bin/env python3

import math
import warnings
import copy

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path


from path_planner import *


DEBUG = True

class Mapping:

    def __init__(self):
        rospy.init_node("lab4_mapping_node")


        # auto update pose
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.update_odometry)


        # self.frontier_goal_service = rospy.Service('frontier_goal', OccupancyGrid, self.get_frontier_goal)
        self.frontier_goal_task_sub = rospy.Subscriber('/task_control/cspace_map', OccupancyGrid, self.calc_frontier)
        # publish the final result for frontier calculation
        self.frontier_goal_pub = rospy.Publisher('/path_planner/frontier_goal', PoseStamped, queue_size=10)

        # for graphing frontier in rviz
        # reuse astar frontier
        self.frontier_graph_pub = rospy.Publisher('/path_planner/frontier', GridCells, queue_size=10)

        # Robot pos
        self.px = 0
        self.py = 0
        self.pth = 0








    ################################ find frontier ################################




    def calc_frontier(self, mapdata):

        initial_frontier_list = Mapping.find_initial_frontier(mapdata)
        print("Calling Expanded")
        expanded_frontier_list = Mapping.inflate_frontier(mapdata, initial_frontier_list)
        print("Calling Erode")
        eroded_frontier_list = Mapping.erode_frontier(mapdata, expanded_frontier_list)

        # Mapping.print_frontier(mapdata, initial_frontier_list)
        # Mapping.print_frontier(mapdata, expanded_frontier_list)
        # Mapping.print_frontier(mapdata, eroded_frontier_list)

        PathPlanner.publish_grid_cell(mapdata, self.frontier_graph_pub, eroded_frontier_list)

        return eroded_frontier_list



    @staticmethod
    def find_initial_frontier(mapdata):

        # list of (x,y) for frontier
        frontier_list = []
        for i,value in enumerate(mapdata.data):
            # print((i,value))
            if value != 0:
                continue
            x,y = PathPlanner.index_to_grid(mapdata,i)
            if Mapping.neighbor_have_unknown(mapdata, x, y):
                frontier_list.append((x,y))

        print(f"\nInitial Frontier List: {frontier_list}")
        # Mapping.print_frontier(mapdata, frontier_list)

        return frontier_list


    @staticmethod
    def neighbor_have_unknown(mapdata, x, y):

        neib_coords = Mapping.neighbors_of_8_all(mapdata, x, y)
        # print(f"\n@neighbor_have_unknown: {neib_coords}")
        for coord in neib_coords:
            index = PathPlanner.grid_to_index(mapdata, coord[0], coord[1])
            value = mapdata.data[index]
            if value == -1:
                # print(f"@neighbor_have_unknown: {(x,y)}, TRUE")

                return True
        
        # print(f"@neighbor_have_unknown: {(x,y)}, FALSE")

        return False


    @staticmethod
    def inflate_frontier(mapdata, frontier_list):
        expanded_frontier_list = copy.deepcopy(frontier_list)

        for f_coord in frontier_list:
            neib_coords = Mapping.neighbors_of_8_all(mapdata, f_coord[0], f_coord[1])
            # print(f"\n@inflate_frontier: {neib_coords}")

            for n_coord in neib_coords:
                index = PathPlanner.grid_to_index(mapdata, n_coord[0], n_coord[1])
                value = mapdata.data[index]
                if value == 0 and n_coord not in expanded_frontier_list:
                    expanded_frontier_list.append(n_coord)

        print(f"\nExpanded Frontier List: {expanded_frontier_list}")
        # Mapping.print_frontier(mapdata, expanded_frontier_list)

        return expanded_frontier_list



    @staticmethod
    def erode_frontier(mapdata, expanded_frontier_list):
        eroded_frontier_list = copy.deepcopy(expanded_frontier_list)

        for f_coord in expanded_frontier_list:
            neib_coords = Mapping.neighbors_of_4_all(mapdata, f_coord[0], f_coord[1])
            for n_coord in neib_coords:
                index = PathPlanner.grid_to_index(mapdata, n_coord[0], n_coord[1])
                value = mapdata.data[index]
                if value == 0 and n_coord not in expanded_frontier_list:
                    if f_coord in eroded_frontier_list:
                        eroded_frontier_list.remove(f_coord)
                    continue

        print(f"\nEroded Frontier List: {eroded_frontier_list}")
        # Mapping.print_frontier(mapdata, eroded_frontier_list)

        return eroded_frontier_list




    ################################ process frontier #################################



    @staticmethod
    def group_frontier(mapdata, frontier_point_list):
        # list of list of (x,y)
        frontier_group_list = []
        for f_coord in frontier_point_list:
            # save the groups that this coord belongs to
            same_group_list = []
            # search with every sorted coord
            for group in frontier_group_list:
                for g_coord in group:
                    g_coord_neib = Mapping.neighbors_of_8_all(mapdata, g_coord[0], g_coord[1])
                    # add to same_group_list
                    if f_coord in g_coord_neib and group not in same_group_list:
                        same_group_list.append(group)

            # remove group that belong from total group list
            # merge all the group into one
            new_group = []
            for same_group in same_group_list:
                frontier_group_list.remove(same_group)
                new_group += same_group
            new_group.append(f_coord)

            frontier_group_list.append(new_group)

        print("\nFinal Group List:")
        for group in frontier_group_list:
            print(group)
        return frontier_group_list





    # def choose_frontier(self, )




















    # region Utils


    #################################### utils ####################################

    @staticmethod
    def neighbors_of_4_all(mapdata, x, y):

        possible_neighbors = []
        in_bound_neighbors = []

        possible_neighbors.append((x+1, y))
        possible_neighbors.append((x-1, y))
        possible_neighbors.append((x, y+1))
        possible_neighbors.append((x, y-1))

        for neighbor in possible_neighbors:
            within_bound = neighbor[0] < mapdata.info.width and \
                neighbor[1] < mapdata.info.height and \
                    min(neighbor[0], neighbor[1]) >= 0
            if within_bound:
                in_bound_neighbors.append(neighbor)
        return in_bound_neighbors


    @staticmethod
    def neighbors_of_8_all(mapdata, x, y):

        possible_neighbors = []
        in_bound_neighbors = []

        possible_neighbors.append((x+1, y))
        possible_neighbors.append((x-1, y))
        possible_neighbors.append((x, y+1))
        possible_neighbors.append((x, y-1))
        possible_neighbors.append((x+1, y+1))
        possible_neighbors.append((x-1, y+1))
        possible_neighbors.append((x+1, y-1))
        possible_neighbors.append((x-1, y-1))

        for neighbor in possible_neighbors:
            within_bound = neighbor[0] < mapdata.info.width and \
                neighbor[1] < mapdata.info.height and \
                    min(neighbor[0], neighbor[1]) >= 0
            if within_bound:
                in_bound_neighbors.append(neighbor)
        return in_bound_neighbors



    @staticmethod
    def print_frontier(mapdata, coord_list):
        print_map = copy.deepcopy(mapdata)
        for coord in coord_list:
            # index = PathPlanner.grid_to_index(mapdata, coord[0], coord[1])
            print_map = PathPlanner.modify_map(print_map, coord[0], coord[1], 5)

        PathPlanner.print_map(print_map)




    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [ quat_orig.x , quat_orig.y , quat_orig.z , quat_orig.w]
        ( roll , pitch , yaw ) = euler_from_quaternion ( quat_list )
        self.pth = yaw




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


    # endregion









    def run(self):
        # mapdata = PathPlanner.get_test_map()
        # PathPlanner.print_map(mapdata)
        # print(f"Map: {mapdata}")

        # mapdata = Mapping.request_map()
        # frontier_list = self.calc_frontier(mapdata)

        # Mapping.group_frontier(mapdata, frontier_list)


        rospy.spin()



if __name__ == '__main__':
    Mapping().run()





