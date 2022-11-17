#!/usr/bin/env python3

import math
import warnings
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped



class PathPlanner:


    
    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node("path_planner")
        
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        # TODO
        self.plan_path_service = rospy.Service('plan_path', GetPlan, self.plan_path)
        
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        # TODO
        self.cspace_pub = rospy.Publisher('/path_planner/cspace', GridCells, queue_size=10)
        
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose the topic names, the message type is GridCells
        # TODO
        self.expanded_pub = rospy.Publisher('/path_planner/expanded', GridCells, queue_size=10)
        self.frontier_pub = rospy.Publisher('/path_planner/frontier', GridCells, queue_size=10)
        self.unexplored_pub = rospy.Publisher('/path_planner/unexplored', GridCells, queue_size=10)
        
        ## Initialize the request counter
        # TODO
        self.request = 0

        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")



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
        newPose = PoseStamped()
        for i in range(len(path)-1):
          # Convert path tuple to world coord (converted is a Point())
          converted = PathPlanner.grid_to_world(mapdata, path[i][0], path[i][1])
          newPose.pose.position.x = converted.x
          newPose.pose.position.y = converted.y
          
          # Create msg header
        #   mapdata.header.seq = self.seq
          mapdata.header.stamp = rospy.Time.now()
        #   mapdata.header.frame_id = 'path'
        #   seq += 1
          
          world.append(newPose) # Append newPose to list of PoseStamped
        return world

    
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

        index = PathPlanner.grid_to_index(mapdata, x, y)
        is_free = not mapdata.data[index]
        # print(f"Walk {(x,y)}")
        # print(within_bound)
        # print(is_free)

        return within_bound & is_free


    @staticmethod
    def is_cell_free(mapdata, x, y):
        index = PathPlanner.grid_to_index(mapdata, x, y)
        is_free = not mapdata.data[index]
        return is_free

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
        try:
            rospy.wait_for_service('static_map') # Block until service is available
            grid = rospy.ServiceProxy('static_map', GetMap)
        except Exception:
            print(f"Error when requesting map\n{Exception}")
            return None

        return grid().map


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
        # TODO
        width = mapdata.info.width
        height = mapdata.info.height
        map_array = mapdata.data
        paded_cell_list = []
        new_map_array = []

        for y in range(height):
            for x in range(width):
                # print(f"padding {(x,y)}")

                if not PathPlanner.is_cell_walkable(mapdata, x, y):
                    new_map_array.append(mapdata.data[PathPlanner.grid_to_index(mapdata, x, y)])
                    # print("continue")
                    continue

                pad_x_range = (x - padding, x + padding+1)
                pad_y_range = (y - padding, y + padding+1)

                neighbor_all_free = True
                for pad_x in range(x - padding, x + padding+1):
                    if not neighbor_all_free:
                        break

                    for pad_y in range(y - padding, y + padding+1):
                        if not neighbor_all_free:
                            break
                        within_bound = pad_x < mapdata.info.width and \
                            pad_y < mapdata.info.height and \
                            min(pad_x,pad_y) >= 0

                        if within_bound:
                            neighbor_all_free &= PathPlanner.is_cell_free(mapdata, pad_x, pad_y)
                            # ocupied = not PathPlanner.is_cell_free(mapdata, pad_x, pad_y)
                            # if ocupied:
                            #     neighbor_all_free = False
                            #     # print(f"neighbor ocupied {(pad_x,pad_y)}")
    
                if neighbor_all_free:
                    new_map_array.append(mapdata.data[PathPlanner.grid_to_index(mapdata, x, y)])
                else:
                    new_map_array.append(100)
                    paded_cell_list.append((x,y))
                    # print("append")
                # if not neighbor_all_free:
                #     mapdata = PathPlanner.modify_map(mapdata, x, y, 100)

        cspace.header = mapdata.header
        cspace.info = mapdata.info
        cspace.data = new_map_array
        PathPlanner.print_map(cspace)

        ## Create a GridCells message and publish it
        # TODO
        grid_cell = GridCells()
        point_list = []
        for cell in paded_cell_list:
            point_list.append(Point(cell[0], cell[1], 0))
            mapdata = PathPlanner.modify_map(mapdata, cell[0], cell[1], 10)
        grid_cell.cells = point_list
        PathPlanner.print_map(mapdata)

        self.cspace_pub.publish(grid_cell)
        ## Return the C-space
        return cspace


    
    def a_star(self, mapdata, start, goal):
        ### REQUIRED CREDIT
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        return []

    
    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")
        return []

        

    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Returning a Path message")

        msg = Path() # Create path msg

        # Create msg header
        # msg.header.seq = self.seq
        msg.header.stamp = rospy.Time.now()
        # msg.header.frame_id = 'map'
        # self.seq += 1

        msg.poses = PathPlanner.path_to_poses(mapdata, path) # Store poses using path_to_poses()

        return msg


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
        cspacedata = self.calc_cspace(mapdata, 1)
        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)



###################################### utils ##################################

    @staticmethod
    def print_map(mapdata):
        width = mapdata.info.width
        height = mapdata.info.height
        map_array = mapdata.data

        for i in range(height):
            for j in range(width):
                # print is up side down, need to print top to bottom
                value = map_array[PathPlanner.grid_to_index(mapdata, j, (height-i-1))]
                if value == 100:
                    print("#", end=" ")
                elif value == 0:
                    print(" ", end=" ")
                else:
                    print("?", end=" ")
            print("")


    @staticmethod
    def modify_map(mapdata, x, y, value):
        # print(f"@ modify_map {(x,y,value)}")
        map_array = list(mapdata.data)
        map_array[PathPlanner.grid_to_index(mapdata, x, y)] = value

        new_mapdata = OccupancyGrid()
        new_mapdata.header = mapdata.header
        new_mapdata.info = mapdata.info
        new_mapdata.data = map_array
        
        return new_mapdata

    @staticmethod
    def get_test_map():
        mapdata = OccupancyGrid()
        # mapdata.info.width = 4
        # mapdata.info.height = 4
        # mapdata.data = (100,0,0,0, 0,0,100,0, 0,0,0,0, 0,0,100,100)

        mapdata.info.width = 3
        mapdata.info.height = 3
        mapdata.data = (100,0,0, 0,0,0, 0,100,100)

        return mapdata

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        mapdata = PathPlanner.request_map()
        # mapdata = PathPlanner.get_test_map()

        # print(mapdata)
        # PathPlanner.print_map(mapdata)

        # print("\n")
        # print(PathPlanner.neighbors_of_8(mapdata, 4, 3))

        # new_mapdata = PathPlanner.modify_map(mapdata, 3, 4, 10)
        # PathPlanner.print_map(new_mapdata)
        
        self.calc_cspace(mapdata, 2)
        # print(PathPlanner.is_cell_free(mapdata, 2,2))

        # rospy.spin()

        # print(PathPlanner.is_cell_walkable(mapdata,0,0))



        
if __name__ == '__main__':
    PathPlanner().run()
