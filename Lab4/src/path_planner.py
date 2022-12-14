#!/usr/bin/env python3

import math
import warnings
import copy
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import Empty, Int32

import priority_queue


CSPACE_LEN = 0.09
# CSPACE_LEN = 0
RESOLUTION = 0.015
# RESOLUTION = 0.05

FREE_VAL = 0
WALL_VAL = 100
CSPACE_VAL = 10
PATH_VAL = 1
UNKNOWN_VAL = -1

TURN_WEIGHT = 5
EXPANDED_WEIGHT = 10

PHASE = None

class PathPlanner:



    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("lab3_path_node")
        
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        self.plan_path_service = rospy.Service('plan_path', GetPlan, self.plan_path)
        
        # self.cspace_map_service = rospy.Service('cspace_map', OccupancyGrid, )

        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.cspace_pub = rospy.Publisher('/path_planner/cspace', GridCells, queue_size=10)
        self.cspace_map_pub = rospy.Publisher('/path_planner/cspace_map', OccupancyGrid, queue_size=10)
        
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose the topic names, the message type is GridCells
        self.expanded_pub = rospy.Publisher('/path_planner/expanded', GridCells, queue_size=10)
        self.frontier_pub = rospy.Publisher('/path_planner/frontier', GridCells, queue_size=10)
        self.unexplored_pub = rospy.Publisher('/path_planner/unexplored', GridCells, queue_size=10)

        self.path_pub = rospy.Publisher('/path_planner/path', Path, queue_size=10)


        self.run_phase_1_sub = rospy.Subscriber('/task_control/run_phase_1', Empty, self.run_phase_1)

        # send a cspace OccupancyGrid
        # request for calculating frontier_goal as one point
        self.frontier_goal_task_pub = rospy.Publisher('/task_control/cspace_map', OccupancyGrid, queue_size=10)

        self.init_phase_sub = rospy.Subscriber('/task_control/init_phase', Int32, self.update_phase)
        self.request_init_phase_pub = rospy.Publisher('/task_control/request_init_phase', Empty, queue_size=10)





        self.cspace_value = math.ceil(CSPACE_LEN / RESOLUTION)
        self.path_partial = 0.8

        # self.phase = 1

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")




    # region helper


    def update_phase(self, msg):
        global PHASE
        PHASE = msg.data
        rospy.loginfo(f"Received msg from /init_phase, updated phase to {msg.data}")
        print(f"Current PHASE: {PHASE}")

    def request_phase(self, msg):
        self.request_init_phase_pub.publish(Empty())


    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        ### REQUIRED CREDIT
        within_bound = x < mapdata.info.width and \
                        y < mapdata.info.height and \
                            min(x,y) >= 0
        if not within_bound:
            warn_str = f"Error: grid out of bound {(x,y)}"
            warnings.warn(warn_str)
            return 0

        index = y * mapdata.info.width + x
        return index


    @staticmethod
    def index_to_grid(mapdata, index):
        width = mapdata.info.width
        height = mapdata.info.height
        map_array = mapdata.data
        if index >= len(map_array):
            warn_str = f"Error: index out of grid {index}"
            warnings.warn(warn_str)
            return (0,0)

        y = index // width
        x = index % height
        return (x,y)

    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """
        ### REQUIRED CREDIT
        return math.sqrt((x1-x2) ** 2 + (y1-y2) ** 2)



    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """
        ### REQUIRED CREDIT
        point = Point()
        point.x = (x+0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
        point.y = (y+0.5) * mapdata.info.resolution + mapdata.info.origin.position.y
        return point


    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        ### REQUIRED CREDIT
        gc_x = int((wp.x - mapdata.info.origin.position.x) / mapdata.info.resolution)
        gc_y = int((wp.y - mapdata.info.origin.position.y) / mapdata.info.resolution)
        return (gc_x, gc_y)


    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        ### REQUIRED CREDIT
        world = []

        for coord in path:
            # Convert path tuple to world coord (converted is a Point())
            newPose = PoseStamped()
            converted = PathPlanner.grid_to_world(mapdata, coord[0], coord[1])
            newPose.pose.position.x = converted.x
            newPose.pose.position.y = converted.y
            newPose.header.frame_id = 'map'

            world.append(newPose)
        return world

    @staticmethod
    def path_to_message(mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        # rospy.loginfo("Returning a Path message")

        msg = Path() # Create path msg

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.poses = PathPlanner.path_to_poses(mapdata, path) # Store poses using path_to_poses()

        return msg


    # TESTED
    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """
        ### REQUIRED CREDIT
        # walkable = False;
        # index = PathPlanner.grid_to_index(mapdata, x, y) # Converts the coordinates to indices in grid
        # if (index < mapdata.info.width and index > 0):
        #   walkable = True

        within_bound = x < mapdata.info.width and \
                        y < mapdata.info.height and \
                            min(x,y) >= 0
        if not within_bound:
            return False
        index = PathPlanner.grid_to_index(mapdata, x, y)
        return not mapdata.data[index]



    @staticmethod
    def is_cell_free(mapdata, x, y):
        index = PathPlanner.grid_to_index(mapdata, x, y)
        is_free = mapdata.data[index] == FREE_VAL or mapdata.data[index] == UNKNOWN_VAL
        return is_free

    @staticmethod
    def is_cell_not_wall(mapdata, x, y):
        within_bound = x < mapdata.info.width and \
                        y < mapdata.info.height and \
                            min(x,y) >= 0

        index = PathPlanner.grid_to_index(mapdata, x, y)
        is_wall = mapdata.data[index] == WALL_VAL
        return within_bound and not is_wall


    @staticmethod
    def is_near_cspace(mapdata, i):
        width = mapdata.info.width
        height = mapdata.info.height
        map_array = mapdata.data
        is_near = False
        # check neighbors
        neib_i_list = [i-1, i+1, i-width-1, i-width, i-width+1, i+width-1, i+width, i+width+1]
        for neib_i in neib_i_list:
            if neib_i > 0 and neib_i < len(map_array):
                is_near |= (map_array[neib_i] == CSPACE_VAL or map_array[neib_i] == WALL_VAL)

        return is_near


    # TESTED
    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """
        ### REQUIRED CREDIT
        possible_neighbors = []
        walkable_neighbors = []
        possible_neighbors.append((x+1, y))
        possible_neighbors.append((x-1, y))
        possible_neighbors.append((x, y+1))
        possible_neighbors.append((x, y-1))

        for neighbor in possible_neighbors:
            if (PathPlanner.is_cell_walkable(mapdata, neighbor[0], neighbor[1])):
                walkable_neighbors.append(neighbor)
        return walkable_neighbors


    # TESTED
    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        ### REQUIRED CREDIT
        possible_neighbors = []
        walkable_neighbors = PathPlanner.neighbors_of_4(mapdata, x, y)

        possible_neighbors.append((x+1, y+1))
        possible_neighbors.append((x-1, y+1))
        possible_neighbors.append((x+1, y-1))
        possible_neighbors.append((x-1, y-1))

        for neighbor in possible_neighbors:
            if (PathPlanner.is_cell_walkable(mapdata, neighbor[0], neighbor[1])):
                walkable_neighbors.append(neighbor)
        return walkable_neighbors


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
        print(f"Running ar PHASE: {PHASE}")
        try:
            if PHASE == 1:
                rospy.wait_for_service('dynamic_map') # Block until service is available
                grid = rospy.ServiceProxy('dynamic_map', GetMap)
            elif PHASE == 3:
                rospy.wait_for_service('static_map') # Block until service is available
                grid = rospy.ServiceProxy('static_map', GetMap)
            else:
                rospy.loginfo(f"ERROR: PHASE not defined: {PHASE}")
        except Exception:
            print(f"Error when requesting map\n{Exception}")
            return None

        return grid().map

    # endregion



    # region cspace

    # TESTED
    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Calculating C-Space")
        cspace = OccupancyGrid()

        ## Go through each cell in the occupancy grid
        ## Inflate the obstacles where necessary
        width = mapdata.info.width
        height = mapdata.info.height
        resolution = mapdata.info.resolution
        paded_cell_list = []
        new_map_array = []

        for y in range(height):
            for x in range(width):

                # is wall, copy
                if not PathPlanner.is_cell_walkable(mapdata, x, y):
                    new_map_array.append(mapdata.data[PathPlanner.grid_to_index(mapdata, x, y)])
                    continue


                neighbor_all_free = True
                for pad_x in range(x - padding, x + padding+1):
                    if not neighbor_all_free:
                        break

                    for pad_y in range(y - padding, y + padding+1):
                        if not neighbor_all_free:
                            break

                        # see if neighbor is wall
                        within_bound = pad_x < mapdata.info.width and \
                            pad_y < mapdata.info.height and \
                            min(pad_x,pad_y) >= 0

                        if within_bound:
                            neighbor_all_free &= PathPlanner.is_cell_free(mapdata, pad_x, pad_y)
                # not neighbor with wall, unchanged
                if neighbor_all_free:
                    new_map_array.append(mapdata.data[PathPlanner.grid_to_index(mapdata, x, y)])
                else:
                    # value = 10 for cspace
                    new_map_array.append(CSPACE_VAL)
                    paded_cell_list.append((x,y))

        cspace.header.frame_id = "map"
        cspace.info = mapdata.info
        cspace.data = new_map_array
        # PathPlanner.print_map(cspace)

        ## Create a GridCells message and publish it
        grid_cell = GridCells()
        grid_cell.header.frame_id = "map"
        grid_cell.cell_width = resolution
        grid_cell.cell_height = resolution
        point_list = []

        for cell in paded_cell_list:
            point_list.append(PathPlanner.grid_to_world(mapdata, cell[0], cell[1]))
        grid_cell.cells = point_list

        self.cspace_pub.publish(grid_cell)
        # self.cspace_map_pub.publish(cspace)
        # give cspace to frontier calc
        # self.frontier_goal_task_pub.publish(cspace)

        ## Return the C-space
        return cspace


    def go_out_of_cspace(self, mapdata, coord):
        """if robot in C space, add a path to go out"""
        search_dir_list = [(1,0), (0,1), (-1,0), (0,-1), (1,1), (1,-1), (-1,-1), (-1,1)]

        for dir in search_dir_list:
            for dist in range(self.cspace_value):
                x = coord[0] + dir[0] * (dist+1)
                y = coord[1] + dir[1] * (dist+1)
                # is cell is wall, discard this direction
                if not PathPlanner.is_cell_not_wall(mapdata, x, y):
                    break
                if PathPlanner.is_cell_walkable(mapdata, x, y):
                    print(f"Out of cspace: {coord} -> {(x,y)}")
                    return (x, y)
        warn_str = f"Error: {coord} is trapped"
        warnings.warn(warn_str)
        return None

    # endregion



    def a_star(self, mapdata, start, goal):
        """
        return list of tuples (x,y) as path
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))

        width = mapdata.info.width
        height = mapdata.info.height
        map_array = mapdata.data

        start_path = [start]
        if not PathPlanner.is_cell_walkable(mapdata, start[0], start[1]):
            warn_str = f"Error: Start grid is not walkable {start}"
            warnings.warn(warn_str)
            out_start = self.go_out_of_cspace(mapdata, start)
            if out_start is None:
                return []
            else:
                start_path = [start, out_start]
                start = out_start

        if not PathPlanner.is_cell_walkable(mapdata, goal[0], goal[1]):
            warn_str = f"Error: Goal grid is not walkable {goal}"
            warnings.warn(warn_str)
            return []

        h_list = [None] * (width * height)
        g_list = [None] * (width * height)
        f_list = [None] * (width * height)
        from_list = [None] * (width * height)
        visited_list = [None] * (width * height)
        all_path_list = [None] * (width * height)
        expanded_cspace = PathPlanner.calc_expanded_cspace(mapdata)

        pq = priority_queue.PriorityQueue()

        start_index = PathPlanner.grid_to_index(mapdata, start[0], start[1])

        # calculate h
        for i in range(len(map_array)):
            if map_array[i] == 0:
                x,y = PathPlanner.index_to_grid(mapdata, i)
                h_list[i] = (PathPlanner.euclidean_distance(x, y, goal[0], goal[1]))
        # print("h calculated")

        # fill in start node
        g_list[start_index] = 0
        f_list[start_index] = g_list[start_index] + h_list[start_index]
        all_path_list[start_index] = start_path
        pq.put(start, f_list[start_index])

        frontier_plot_list = []
        visited_plot_list = []

        while not pq.empty():
            # pop current from frontier
            current = pq.get()
            current_i = PathPlanner.grid_to_index(mapdata, current[0], current[1])

            # retrieve final path
            if current == goal:
                rospy.loginfo("A* Path Found")

                path_list = all_path_list[current_i]
                print(f"Path: {path_list}")
                # PathPlanner.print_path(mapdata, path_list)
                # PathPlanner.print_num_value(mapdata, g_list)

                # path_msg = PathPlanner.path_to_message(mapdata, path_list)

                # self.path_pub.publish(path_msg)
                # PathPlanner.print_num_value(mapdata, g_list)
                rospy.sleep(0.5)
                return path_list


            # get neighbors
            # neighbors = PathPlanner.neighbors_of_4(mapdata, current[0], current[1])
            neighbors = PathPlanner.neighbors_of_8(mapdata, current[0], current[1])

            # process neighbors
            for neib in neighbors:
                neib_i = PathPlanner.grid_to_index(mapdata,neib[0], neib[1])
                if visited_list[neib_i]:
                    continue

                # construct path from start to neib
                neighbor_path = copy.deepcopy(all_path_list[current_i])
                neighbor_path.append(neib)

                path_value = neighbor_path
                g_value = PathPlanner.calc_g_value(mapdata, neighbor_path, expanded_cspace)
                x,y = PathPlanner.index_to_grid(mapdata, neib_i)
                # h_list[neib_i] = (PathPlanner.euclidean_distance(x, y, goal[0], goal[1]))
                # f_value = g_value + h_list[neib_i]
                h_value = PathPlanner.calc_h_value(neighbor_path, x, y, goal[0], goal[1])
                f_value = g_value + h_value

                # if successfully put, return True
                if pq.put(neib, f_value):
                    from_list[neib_i] = current
                    g_list[neib_i] = g_value
                    f_list[neib_i] = f_value
                    all_path_list[neib_i] = path_value
                    frontier_plot_list.append(neib)

            # current add to visited
            visited_list[current_i] = True
            visited_plot_list.append(current)

            # # plot process in Rviz
            # PathPlanner.publish_grid_cell(mapdata, self.frontier_pub, frontier_plot_list)
            # PathPlanner.publish_grid_cell(mapdata, self.expanded_pub, visited_plot_list)

            # rospy.sleep(0.02)
        return []


    @staticmethod
    def calc_g_value(mapdata, from_path, expanded_cspace):
        """calculat g value based on real distance and number of turns"""
        turn_weight = TURN_WEIGHT
        expanded_cspace_weight = EXPANDED_WEIGHT
        walk_distance = 0
        turn_count = 0
        for i in range(1,len(from_path)):
            last = from_path[i-1]
            this = from_path[i]
            # calculate distance walked
            this_dist = math.sqrt((this[0]-last[0])**2 + (this[1]-last[1])**2)
            walk_distance += this_dist
            # add weight for expanded cspace
            this_i = PathPlanner.grid_to_index(mapdata, this[0], this[1])
            walk_distance += expanded_cspace[this_i] * expanded_cspace_weight

            if i == len(from_path)-1:
                continue
            # calculate turns
            next = from_path[i+1]
            if not (this[0]-last[0] == next[0]-this[0] and \
                this[1]-last[1] == next[1]-this[1]):
                turn_count += 1

        g_value = walk_distance + turn_weight * turn_count
        return g_value


    @staticmethod
    def calc_h_value(from_path, neib_x, neib_y, goal_x, goal_y):
        dist = PathPlanner.euclidean_distance(neib_x, neib_y, goal_x, goal_y)
        if len(from_path) < 2:
            return dist

        turn_weight = 10

        this = from_path[-1]
        last = from_path[-2]
        next = (neib_x, neib_y)
        if not (this[0]-last[0] == next[0]-this[0] and \
                this[1]-last[1] == next[1]-this[1]):
            dist += turn_weight
        return dist


    @staticmethod
    def calc_expanded_cspace(mapdata):
        """
        give higher cost to cell around cspace
        return a map of if cell is near cspace
        """
        width = mapdata.info.width
        height = mapdata.info.height
        map_array = mapdata.data
        expanded_cspace = [0] * len(map_array)
        for i,val in enumerate(map_array):
            if val == FREE_VAL:
                expanded_cspace[i] = int(PathPlanner.is_near_cspace(mapdata, i))
        
        # print_map = copy.deepcopy(mapdata)
        # print_map.data = expanded_cspace
        # PathPlanner.print_map(print_map)
        return expanded_cspace

    @staticmethod
    def get_partial_path(path, partial):
        partial_path_length = math.floor(len(path) * partial)
        return path[:partial_path_length]

    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        # rospy.loginfo("Optimizing path")

        if len(path) == 0:
            warnings.warn("Path is empty")
            return []

        new_path = []
        new_path.append(path[0])
        for i in range(1,len(path)-1):
            last = path[i-1]
            this = path[i]
            next = path[i+1]
            dx1 = this[0]-last[0]
            dy1 = this[1]-last[1]
            dx2 = next[0]-this[0]
            dy2 = next[1]-this[1]

            # there is a turn
            if dx1 - dx2 != dy1 - dy2:
                new_path.append(this)

        new_path.append(path[-1])
        print(f"Optimized Path: {new_path}")
        return new_path






    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req
        """
        ## Request the map
        ## In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        ## Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, self.cspace_value)
        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        ## Optimize waypoints
        if PHASE == 1:
            partial_path = PathPlanner.get_partial_path(path, self.path_partial)
        elif PHASE == 3:
            partial_path = path
        else:
            rospy.loginfo(f"ERROR: PHASE not defined: {PHASE}")
            partial_path = path
        waypoints = PathPlanner.optimize_path(partial_path)
        ## Return a Path message
        path_msg = PathPlanner.path_to_message(mapdata, waypoints)

        self.path_pub.publish(path_msg)
        # print(f"\nReturn from Service: \n{path_msg}")
        return path_msg




    def run_phase_1(self, msg=None):
        """Start phase 1"""
        rospy.loginfo("Recieved Command from Drive, running phase 1")
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            warnings.warn("Empty Map")
            return
        cspace_map = self.calc_cspace(mapdata, self.cspace_value)
        self.frontier_goal_task_pub.publish(cspace_map)
        rospy.loginfo("Sent Cspace to Mapping, Calling calc_frontier")



###################################### utils ##################################

    @staticmethod
    def print_map(mapdata):
        """Print map in terminal"""
        width = mapdata.info.width
        height = mapdata.info.height
        map_array = mapdata.data

        for i in range(height):
            for j in range(width):
                # print is up side down, need to print top to bottom
                value = map_array[PathPlanner.grid_to_index(mapdata, j, (height-i-1))]
                if value == WALL_VAL:
                    print("#", end=" ")
                elif value == FREE_VAL:
                    print(" ", end=" ")
                elif value == UNKNOWN_VAL:
                    print("+", end=" ")
                elif value == CSPACE_VAL:
                    print("-", end=" ")
                elif value == 5:
                    print("@", end=" ")
                elif value == PATH_VAL:
                    print("&", end=" ")
                else:
                    print("?", end=" ")
            print("")

    @staticmethod
    def print_num_value(mapdata, value_list):
        """Print a numerical value for the whole map in terminal"""
        width = mapdata.info.width
        height = mapdata.info.height

        for i in range(height):
            for j in range(width):
                # print is up side down, need to print top to bottom
                value = value_list[PathPlanner.grid_to_index(mapdata, j, (height-i-1))]
                if value is None:
                    print("-", end="\t")
                # elif value == 0:
                #     print("0.0", end="\t")
                else:
                    print(round(value,1), end="\t")
                # if value == 100:
                #     print("#", end=" ")
                # elif value == 0:
                #     print(" ", end=" ")
                # else:
                #     print("?", end=" ")
            print("\n", end="")


    @staticmethod
    def print_path(mapdata, path_list):
        print_map = copy.deepcopy(mapdata)
        for coord in path_list:
            print_map = PathPlanner.modify_map(print_map, coord[0], coord[1], 1)
        PathPlanner.print_map(print_map)


    @staticmethod
    def modify_map(mapdata, x, y, value):
        # print(f"@ modify_map {(x,y,value)}")
        map_array = list(copy.deepcopy(mapdata.data))
        map_array[PathPlanner.grid_to_index(mapdata, x, y)] = value

        new_mapdata = OccupancyGrid()
        new_mapdata.header = mapdata.header
        new_mapdata.info = mapdata.info
        new_mapdata.data = map_array

        return new_mapdata

    @staticmethod
    def get_test_map():
        mapdata = OccupancyGrid()
        mapdata.info.width = 5
        mapdata.info.height = 5

        a = WALL_VAL
        b = CSPACE_VAL
        c = UNKNOWN_VAL
        # mapdata.data = (-1,-1,0,0,0, 0,0,0,-1,0, 0,100,0,0,0, 0,0,0,0,0, 0,0,0,100,100)
        mapdata.data = (0,a,a,a,0,\
                        0,b,b,b,0,\
                        0,0,0,0,0,\
                        0,0,0,0,0,\
                        0,0,0,0,0\
                        )

        return mapdata


    @staticmethod
    def publish_grid_cell(mapdata, publisher, grid_list):
        grid_cell_msg = GridCells()
        grid_cell_msg.header.frame_id = "map"
        grid_cell_msg.cell_width = mapdata.info.resolution
        grid_cell_msg.cell_height = mapdata.info.resolution
        point_list = []
        for coord in grid_list:
            point_list.append(PathPlanner.grid_to_world(mapdata, coord[0], coord[1]))

        # print(f"Published Points: {point_list}")
        grid_cell_msg.cells = point_list

        publisher.publish(grid_cell_msg)
        rospy.sleep(0.005)





    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.loginfo("Path Planner sleeping")
        self.request_init_phase_pub.publish(Empty())
        rospy.sleep(1)
        rospy.loginfo("Wake up")

        # mapdata = PathPlanner.request_map()
        # self.calc_cspace(mapdata, self.cspace_value)
        self.run_phase_1()
        print(f"CSpace padding: {self.cspace_value}")


        rospy.spin()


        # mapdata = PathPlanner.get_test_map()
        # PathPlanner.print_map(mapdata)
        # self.a_star(mapdata, (0,2), (4,3))




if __name__ == '__main__':
    PathPlanner().run()
