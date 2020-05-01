#!/usr/bin/env python3

import sys
import numpy as np
import matplotlib.pyplot as plt
import random as rand
import time

MAGIC_NUMBER = 100 # it helps visualising the colors on plot
moves = ["UR", "U", "UL", "L", "DL", "D", "DR", "R"] # ordered circularily for perimeter following mode
moves_count = 8
moves_indexes = { "UR" : 1,
                  "U" : 2, 
                  "UL" : 3,
                  "L" : 4,
                  "DL" : 5,
                  "D" : 6,
                  "DR" : 7,
                  "R" : 0 }

# global variables, initialized with random values. they will be overwritten at the read phase
H = 0 # map's height
W = 0 # map's width
N = 0 # num of robots
R = 0 # robots' communication radius
map = np.zeros((H, W))
robots = dict()

class Robot:
    def __init__(self, id, pos_x, pos_y):
        self.id = id
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.N_on = []
        self.N_off = []
        self.N_u = []
        self.P_obs = []
        self.map_info = []

        own_island = set()
        own_island.add(id)
        self.map_info.append(own_island)

        # perimeter following mode
        self.pfm_start_x = -1 # positon when starting pfm
        self.pfm_start_y = -1

        # checkpoint, robot A trying to reach robot B
        self.goal_path = [] # the path that robot A should follow to reach robot B
        self.goal_x = -1 # position when robot B was first sensed
        self.goal_y = -1 
        self.obs_start_x = -1 # position when encountered an obstacle
        self.obs_start_y = -1 # reaching this instead of a goal_path coordonate will conclude that the other robnot is on other island REWORK HERE
        self.goal_id = -1 # robot's B id

def read_input():
    global H, W, N, R, map, robots

    f = open(sys.argv[1], "rb")
    lines = f.read().decode("utf-8").split('\r\n')
    
    # define map
    H = int(lines[0])
    W = int(lines[1])
    map = np.zeros((H, W))

    # populate the margins
    populate_map_margins()

    # define robots
    N = int(lines[2])
    R = int(lines[3])
    for i in range(4, 4 + N):
        pos = lines[i].split(" ") # robot_H robot_W
        robots[i - 3] = Robot(i - 3, int(pos[0]), int(pos[1])) # i - 3 because robots' indexes start from 1
        if (map[int(pos[0])][int(pos[1])] == 0):
            map[int(pos[0])][int(pos[1])] = i - 3 + MAGIC_NUMBER # place robot on map
        else:
            print("Wrong input file. Robot overlapping: %s" % lines[i])
            sys.exit(2)

    # complete map with obstacles
    for i in range(4 + N, len(lines)):
        place_obstacles(lines[i])

    # possibility of adding additional robots
    random_robots_N = int(sys.argv[3])
    if (random_robots_N != 0):
        for i in range(N + 1, N + random_robots_N + 1):
            while(1):
                random_pos_x = rand.randrange(H)
                random_pos_y = rand.randrange(W)
                if (map[random_pos_x][random_pos_y] == 0):
                    robots[i] = Robot(i, random_pos_x, random_pos_y)
                    map[random_pos_x][random_pos_y] = i + MAGIC_NUMBER
                    break
        N += random_robots_N

    f.close()

def place_obstacles(obstacle):
    obstacles = obstacle.split(" ")
    if (obstacles[0] == 'H'): # horizontal line obstacle
        line = int(obstacles[1])
        if (np.amax(map[line]) > 0): 
            print("Wrong input file. Invalid obstacle: %s" % obstacle)
            sys.exit(2)
        else:
            map[line] = np.full((W), -1 - MAGIC_NUMBER)
    elif (obstacles[0] == 'V'): # vertical line obstacle
        line = int(obstacles[1])
        if (np.amax(map[:, line]) > 0):
            print("Wrong input file. Invalid obstacle: %s" % obstacle)
            sys.exit(2)
        else:
            map[:, line] = np.full((H), -1 - MAGIC_NUMBER)
    elif (obstacles[0] == 'S'): # single point obstacle
        if (map[int(obstacles[1]), int(obstacles[2])] <= 0):
            map[int(obstacles[1]), int(obstacles[2])] = -1 - MAGIC_NUMBER
        else:
            print("Wrong input file. Invalid obstacle: %s" % obstacle)
            sys.exit(2)
    elif (obstacles[0] == 'R'): # rectangle obstacle (full of water)
        x1 = int(obstacles[1])
        x2 = int(obstacles[2])
        y1 = int(obstacles[3])
        y2 = int(obstacles[4])

        for line in range(x1, x2 + 1):
            if (np.amax(map[line][y1:y2+1]) > 0):
                print("Wrong input file. Invalid obstacle: %s" % obstacle)
                sys.exit(2)
            else:
                map[line][y1:y2+1] = np.full((y2 - y1 + 1), -1 - MAGIC_NUMBER)

def populate_map_margins():
    place_obstacles("V 0")
    place_obstacles("H 0")
    place_obstacles("V " + str(W - 1))
    place_obstacles("H " + str(H - 1))
 
def print_map_state():
    global map
    plt.imshow(map, cmap="cividis")
    plt.show()

# returns a list of robots in the communication range for a given robot
def discover_robots(robot):
    start_x = robot.pos_x - R
    if (start_x < 1):
        start_x = 1
    stop_x = robot.pos_x + R
    if (stop_x >= H - 1):
        stop_x = H - 2
    start_y = robot.pos_y - R
    if (start_y < 1):
        start_y = 1
    stop_y = robot.pos_y + R
    if (stop_y >= W - 1):
        stop_y = W - 2
    
    N_r = []
    for i in range(start_x, stop_x + 1):
        for j in range(start_y, stop_y + 1):
            if (i == robot.pos_x and j == robot.pos_y):
                continue
            robot_id = map[i][j] - MAGIC_NUMBER
            if (robot_id > 0):
                N_r.append(robot_id)
    return N_r

def move_robot(direction, robot):
    global map
    if (direction == "L"):
        map[robot.pos_x][robot.pos_y - 1] = robot.id + MAGIC_NUMBER
        map[robot.pos_x][robot.pos_y] = 0
        robot.pos_y -= 1
    elif (direction == "R"):
        map[robot.pos_x][robot.pos_y + 1] = robot.id + MAGIC_NUMBER
        map[robot.pos_x][robot.pos_y] = 0
        robot.pos_y += 1
    elif (direction == "D"):
        map[robot.pos_x + 1][robot.pos_y] = robot.id + MAGIC_NUMBER
        map[robot.pos_x][robot.pos_y] = 0
        robot.pos_x += 1
    elif (direction == "U"):
        map[robot.pos_x - 1][robot.pos_y] = robot.id + MAGIC_NUMBER
        map[robot.pos_x][robot.pos_y] = 0
        robot.pos_x -= 1
    elif (direction == "DL"):
        map[robot.pos_x + 1][robot.pos_y - 1] = robot.id + MAGIC_NUMBER
        map[robot.pos_x][robot.pos_y] = 0
        robot.pos_x += 1
        robot.pos_y -= 1
    elif (direction == "DR"):
        map[robot.pos_x + 1][robot.pos_y + 1] = robot.id + MAGIC_NUMBER
        map[robot.pos_x][robot.pos_y] = 0
        robot.pos_y += 1
        robot.pos_x += 1
    elif (direction == "UL"):
        map[robot.pos_x - 1][robot.pos_y - 1] = robot.id + MAGIC_NUMBER
        map[robot.pos_x][robot.pos_y] = 0
        robot.pos_x -= 1
        robot.pos_y -= 1
    elif (direction == "UR"):
        map[robot.pos_x - 1][robot.pos_y + 1] = robot.id + MAGIC_NUMBER
        map[robot.pos_x][robot.pos_y] = 0
        robot.pos_x -= 1
        robot.pos_y += 1

def verify_move(direction, robot):
    if (direction == "L"):
        if (map[robot.pos_x][robot.pos_y - 1] >= 0):
            return True
    elif (direction == "R"):
        if (map[robot.pos_x][robot.pos_y + 1] >= 0):
            return True
    elif (direction == "D"):
        if (map[robot.pos_x + 1][robot.pos_y] >= 0):
            return True
    elif (direction == "U"):
        if (map[robot.pos_x - 1][robot.pos_y] >= 0):
            return True
    elif (direction == "UL"):
        if (map[robot.pos_x - 1][robot.pos_y - 1] >= 0):
            return True
    elif (direction == "UR"):
        if (map[robot.pos_x - 1][robot.pos_y + 1] >= 0):
            return True
    elif (direction == "DL"):
        if (map[robot.pos_x + 1][robot.pos_y - 1] >= 0):
            return True
    elif (direction == "DR"):
        if (map[robot.pos_x + 1][robot.pos_y + 1] >= 0):
            return True
    else:
        return False

def move_randomly(robot):
    move_index = rand.randrange(moves_count)
    for i in range(moves_count):
        move_attempt = (move_index + i) % moves_count
        if (verify_move(moves[move_attempt], robot)):
            move_robot(moves[move_attempt], robot)
            break

def set_adjacent_obstacles(robot):
    robot.P_obs = [] # clear previous data

    # V and H
    if (map[robot.pos_x - 1][robot.pos_y] == -1 - MAGIC_NUMBER):
        robot.P_obs.append((robot.pos_x - 1, robot.pos_y))
    if (map[robot.pos_x + 1][robot.pos_y] == -1 - MAGIC_NUMBER):
        robot.P_obs.append((robot.pos_x + 1, robot.pos_y))
    if (map[robot.pos_x][robot.pos_y - 1] == -1 - MAGIC_NUMBER):
        robot.P_obs.append((robot.pos_x, robot.pos_y - 1))
    if (map[robot.pos_x][robot.pos_y + 1] == -1 - MAGIC_NUMBER):
        robot.P_obs.append((robot.pos_x, robot.pos_y + 1))

    # diagonal
    if (map[robot.pos_x - 1][robot.pos_y - 1] == -1 - MAGIC_NUMBER):
        robot.P_obs.append((robot.pos_x - 1, robot.pos_y - 1))
    if (map[robot.pos_x + 1][robot.pos_y + 1] == -1 - MAGIC_NUMBER):
        robot.P_obs.append((robot.pos_x + 1, robot.pos_y + 1))
    if (map[robot.pos_x + 1][robot.pos_y - 1] == -1 - MAGIC_NUMBER):
        robot.P_obs.append((robot.pos_x + 1, robot.pos_y - 1))
    if (map[robot.pos_x - 1][robot.pos_y + 1] == -1 - MAGIC_NUMBER):
        robot.P_obs.append((robot.pos_x - 1, robot.pos_y + 1))

# returns the orientation of the obstacle from the robot's point of view (i.e. if r is at (0,0) and at (2, 2) there's water, it will return DR)
def orientation_from_point(robot_pos, point_pos):
    point_pos_x = point_pos[0]
    point_pos_y = point_pos[1]
    robot_pos_x = robot_pos[0]
    robot_pos_y = robot_pos[1]

    dist_x = robot_pos_x - point_pos_x
    dist_y = robot_pos_y - point_pos_y
    dir_x = "N" # U D
    dir_y = "N" # L R

    if dist_x < 0:
        dir_x = "D"
    elif dist_x > 0:
        dir_x = "U"
    if dist_y < 0:
        dir_y = "R"
    elif dist_y > 0:
        dir_y = "L"
    if dir_x == "N":
        return dir_y
    elif dir_y == "N":
        return dir_x
    else:
        return dir_x + dir_y # they can't be both N/A

def test_adjacent_obstacles():
    global robots
    global map
    while (1):
        robot_index = rand.randrange(1, N + 1) # robots' indexes start from 1
        current_robot = robots.get(robot_index)
        move_randomly(current_robot)
        set_adjacent_obstacles(current_robot)
        if (current_robot.P_obs != []):
            print("Robot %d: %s" % (current_robot.id, current_robot.P_obs))
            # print_map_state()

def perimeter_following_mode(robot):
    global moves_indexes
    for obstacle in robot.P_obs:
        orientation = orientation_from_point((robot.pos_x, robot.pos_y), obstacle) # the obstacle is <orientation> (i.e. Up Right) from robot
        move_index = moves_indexes[orientation] # direction to follow in order to keep it on the right
        direction = moves[move_index]
        if (verify_move(direction, robot)): # it can move towards this direction
            if (robot.pfm_start_x == -1): # just started the perimeter following mode
                robot.pfm_start_x = robot.pos_x
                robot.pfm_start_y = robot.pos_y
            elif (robot.pfm_start_x == robot.pos_x and robot.pfm_start_y == robot.pos_y): # returns to the same location
                robot.pfm_start_x = -1 # we leave the pfm
                robot.pfm_start_y = -1
                move_randomly(robot) # and move randomly
                return False
            move_robot(direction, robot)
            return True

def highlight_path(path):
    global map
    for pos in path:
        pos_x = pos[0]
        pos_y = pos[1]
        map[pos_x][pos_y] = MAGIC_NUMBER / 2

def test_pfm():
    global map
    global robots
    path = []
    while (1):
        robot_index = rand.randrange(1, N + 1) # robots' indexes start from 1
        current_robot = robots.get(robot_index)
        set_adjacent_obstacles(current_robot)
        if (current_robot.P_obs != []):
            print("Robot %d at initial position: (%d, %d)" % (current_robot.id, current_robot.pos_x, current_robot.pos_y))
            path.append((current_robot.pos_x, current_robot.pos_y))
            print_map_state()
            while(1):
                if (perimeter_following_mode(current_robot)):
                    set_adjacent_obstacles(current_robot)
                    path.append((current_robot.pos_x, current_robot.pos_y))
                else:
                    print("Robot %d at one random cell away: (%d, %d)" % (current_robot.id, current_robot.pos_x, current_robot.pos_y))
                    highlight_path(path)
                    print_map_state()
                    return
        else:
            move_randomly(current_robot)

def calculate_path(start, stop):
    start_x = start[0]
    start_y = start[1]
    stop_x = stop[0]
    stop_y = stop[1]

    path = []

    while(1):
        if (start_x == stop_x and start_y == stop_y):
            break
        orientation = orientation_from_point((start_x, start_y), (stop_x, stop_y))
        if (orientation == "UR"):
            start_x -= 1
            start_y += 1
        elif (orientation == "U"):
            start_x -= 1
        elif (orientation == "UL"):
            start_x -= 1
            start_y -= 1
        elif (orientation == "L"):
            start_y -= 1
        elif (orientation == "DL"):
            start_x += 1
            start_y -= 1
        elif (orientation == "D"):
            start_x += 1
        elif (orientation == "DR"):
            start_x += 1
            start_y += 1
        elif (orientation == "R"):
            start_y += 1
        path.append((start_x, start_y))
    
    return path

def test_calculate_path(start, stop):
    path = calculate_path(start, stop)
    highlight_path(path)
    map[start[0]][start[1]] = -MAGIC_NUMBER / 2 
    map[stop[0]][stop[1]] = -MAGIC_NUMBER / 2
    print_map_state()

# only for N_u, N_on and N_off
def update_neighbour_info(robot, is_neighbour):
    if (is_neighbour):
        robot.N_on.append(robot.goal_id)
        # place this robot inside your island in map's info
        for island_index in range(len(robot.map_info)):
            if (robot.id in robot.map_info[island_index]):
                robot.map_info[island_index].add(robot.goal_id)
                break
    else:
        robot.N_off.append(robot.goal_id)

        # optimization - don't add to your islands if you already have that one added
        check = True
        for island in robot.map_info:
            if (robot.goal_id in island):
                check = False
                break
        if (check): # create new island as you don't know where exactly is this robot
            new_island = set()
            new_island.add(robot.goal_id)
            robot.map_info.append(new_island)

    del robot.N_u[0]
    robot.goal_id = -1

# only when encountering an obstacle
def move_towards_goal_line(robot):
    global moves_indexes
    for obstacle in robot.P_obs:
        orientation = orientation_from_point((robot.pos_x, robot.pos_y), obstacle) # the obstacle is <orientation> (i.e. Up Right) from robot
        move_index = moves_indexes[orientation] # direction to follow in order to keep it on the right
        direction = moves[move_index]
        if (verify_move(direction, robot)): # it can move towards this direction
            move_robot(direction, robot)
            break

# update N_on/N_off and N_u by island !* ONLY for same island robots *!
def update_by_island(robot, island):

    for id in island:
        if (id not in robot.N_on and id != robot.id): # on the same island and the robot didn't know yet
            robot.N_on.append(id)
            # every N_off of this robot is also your N_off
            for stranger in robots[id].N_off:
                if (stranger not in robot.N_off):
                    robot.N_off.append(stranger)
                    if (stranger in robot.N_u): # if it aims to reach it already
                        if (robot.goal_id == stranger):
                            robot.goal_id = -1
                        index = robot.N_u.index(stranger)
                        del robot.N_u[index]

        if (id in robot.N_u): # clear the exploring queue
            index = robot.N_u.index(id)
            del robot.N_u[index]

        if (robot.goal_id == id): # the robot was already following him, so it must stop
            robot.goal_id = -1

# update neighbours
def notify_unreachable_robots(r1, r2):
    # for r1
    if (r1.id in r2.N_off): # not neighbours
        for id in r2.N_on:
            if (id not in r1.N_off):
                r1.N_off.append(id)
                if (r1.goal_id == id):
                    r1.goal_id = -1
                if (id in r1.N_u):
                    index = r1.N_u.index(id)
                    del r1.N_u[index]
    elif (r1.id in r2.N_on): # neighbours
        for id in r2.N_off:
            if (id not in r1.N_off):
                r1.N_off.append(id)
                if (r1.goal_id == id):
                    r1.goal_id = -1
                if (id in r1.N_u):
                    index = r1.N_u.index(id)
                    del r1.N_u[index]
    if (r1.id not in r2.N_on): # we don't know yet if they are neighbours
        for unk_r in r2.N_off:
            if (unk_r != r1.id and unk_r not in r1.N_on and unk_r not in r1.N_off and unk_r not in r1.N_u): # not reached by r1 yet
                r1.N_u.append(unk_r)
    
    # for r2
    if (r2.id in r1.N_off): # not neighbours
        for id in r1.N_on:
            if (id not in r2.N_off):
                r2.N_off.append(id)
            if (r2.goal_id == id):
                    r2.goal_id = -1
            if (id in r2.N_u):
                index = r2.N_u.index(id)
                del r2.N_u[index]
    elif (r2.id in r1.N_on):
        for id in r1.N_off:
            if (id not in r2.N_off):
                r2.N_off.append(id)
                if (r2.goal_id == id):
                    r2.goal_id = -1
                if (id in r2.N_u):
                    index = r2.N_u.index(id)
                    del r2.N_u[index]
    
    if (r2.id not in r1.N_on): # we don't know yet if they are neighbours
        for unk_r in r1.N_off:
            if (unk_r != r2.id and unk_r not in r2.N_on and unk_r not in r2.N_off and unk_r not in r2.N_u): # not reached by r2 yet
                r2.N_u.append(unk_r)

def update_map_info(r1, r2, merged_islands):
    r1.map_info = merged_islands
    r2.map_info = merged_islands

    for island in merged_islands:

        # update r1 info
        if (r1.id in island): # r1's map island, deal with N_on
            update_by_island(r1, island)
        # update r2 info
        if (r2.id in island): # r2's map island, deal with N_on
            update_by_island(r2, island)

    # if r1 and r2 are not neighbours, check for possibly neighbours for each other
    # this way, they will be able to determine if someone 3 islands long is neighbour or not
    notify_unreachable_robots(r1, r2)

# merge all islands within a list (no duplicates)
def compute_merging(merged_islands):
    for i in range(len(merged_islands)):
        for j in range(i + 1, len(merged_islands)):
            if (merged_islands[i] & merged_islands[j]): # they have at least one common element
                common_island = set().union(merged_islands[i], merged_islands[j]) # merge islands
                del merged_islands[j]
                merged_islands[i] = common_island
                return True, merged_islands
    return False, merged_islands

def merge_alg34(r1, r2):
    merged_islands = []
    if (r1.map_info == r2.map_info):
        notify_unreachable_robots(r1, r2)
        return
    merged_islands = r1.map_info + r2.map_info # concatenate islands
    while(1):
        result, merged_islands = compute_merging(merged_islands)
        if (not result):
            update_map_info(r1, r2, merged_islands)
            break

def checkpoint_alg2(robot):
    # mark that the robot is no longer in pfm
    robot.pfm_start_x = -1
    robot.pfm_start_y = -1

    if (robot.goal_id == -1): # just started the algorithm for a new robot
        robot.goal_id = robot.N_u[0] # get the first id in its unknown list
        goal_robot = robots.get(robot.goal_id) # robot object that needs to be reached
        robot.goal_x = goal_robot.pos_x
        robot.goal_y = goal_robot.pos_y
        robot.goal_path = calculate_path((robot.pos_x, robot.pos_y), (robot.goal_x, robot.goal_y))
        robot.obs_start_x = -1
        robot.obs_start_y = -1

    if (robot.pos_x == robot.goal_x and robot.pos_y == robot.goal_y): # already at destination. needed as robots can overlap
        update_neighbour_info(robot, True)
        move_randomly(robot) # it can do anything here, but decided to move randomly
        return

    if (robot.obs_start_x == -1): # didn't encounter any obstacles until now
        next_step = robot.goal_path[0] # get the next step
        del robot.goal_path[0] # and delete it from your list
        direction = orientation_from_point((robot.pos_x, robot.pos_y), next_step) # aim to go in this direction

        if (verify_move(direction, robot)):
            move_robot(direction, robot)
            if (robot.pos_x == robot.goal_x and robot.pos_y == robot.goal_y):
                update_neighbour_info(robot, True)
        else: # we just encountered an obstacle
            robot.obs_start_x = robot.pos_x
            robot.obs_start_y = robot.pos_y
            move_towards_goal_line(robot) # the robot just moved, keeping the obstacle on the right
    else: # we are trying to get rid of the obstacle
        move_towards_goal_line(robot)

        if (robot.pos_x == robot.obs_start_x and robot.pos_y == robot.obs_start_y): # the robot is on other island
            update_neighbour_info(robot, False)
        else:
            # check if we returned on the straight line
            current_pos = (robot.pos_x, robot.pos_y)
            if current_pos in robot.goal_path:
                robot.obs_start_x = -1
                robot.obs_start_y = -1
                position_index = robot.goal_path.index(current_pos) # get the current position's index
                del robot.goal_path[0:position_index + 1] # delete all the blocked steps, including the current one

def move_alg5(robot):
    if (robot.N_u == []):
        if (robot.P_obs == []):
            move_randomly(robot)
        else:
            perimeter_following_mode(robot)
    else:
        checkpoint_alg2(robot)

# for creating graphs
def store_stats(iteration, elapsed_time):
    root_name = str(sys.argv[1])
    root_name = root_name.replace('.txt', '')
    root_name = root_name + "_stats.txt"
    
    f = open(root_name, "a+")
    stats = str(N) + " " + str(iteration) + " " + str(elapsed_time) + "\n"
    f.write(stats)
    f.close()    

def print_results(iteration, elapsed_time):
    global robots
    print("Map structure for %d robots: %s" % (N, robots[1].map_info))
    print("Number of iterations: %d and elapsed time: %s" %(iteration, time.strftime("%H:%M:%S", time.gmtime(elapsed_time))))
    
    # print_map_state()

# check if all the robots agreed with the map's structure
def check_end():
    global robots

    random_map_info = robots[1].map_info # ids start at 1

    for robot in robots.values():
        if (len(robot.N_on) + len(robot.N_off) != N - 1): # we should verify all robots
            return False
        if (robot.map_info != random_map_info): # same map structure
            return False
    
    return True

def main():
    global map
    global robots

    if len(sys.argv) != 4:
        print("Usage: robots.py <INPUT_FILE> <iter> <random_robots>")
        sys.exit(2)
    
    read_input()

    # preview initial map state
    print_map_state()

    tested = []
    iteration = 1
    iter_threshold = int(sys.argv[2])
    start_time = time.time()

    # main_alg1
    while(1):

        # see details on route
        if (iter_threshold != 0):
            if (iteration % iter_threshold == 0):
                for robot in robots.values(): # check what robots are ready
                    # print("%d: On: %s Off: %s U: %s" % (robot.id, robot.N_on, robot.N_off, robot.N_u))
                    # print("%d: %s" % (robot.id, robot.map_info))
                    if (robot.id not in tested and len(robot.N_on) + len(robot.N_off) == N - 1):
                        tested.append(robot.id)
                print("Iteration %d. %d/%d fully-discovered robots: %s" % (iteration, len(tested), N, tested))
                # print_map_state()
        iteration += 1

        for robot in robots.values():
            N_r = discover_robots(robot) # discover robots in communication range
            for s in N_r:
                if (s in robot.N_on or s in robot.N_off): # s is on the same island or on a different one (known fact for robot)
                    merge_alg34(robot, robots[s])
                elif (s in robot.N_u): # s robot's island was already sensed and is or will be processed
                    continue
                else:
                    robot.N_u.append(s) # set s as unknown
            set_adjacent_obstacles(robot)
            move_alg5(robot)
        
        # after an iteration, check if we finished
        if (check_end()):
            elapsed_time = time.time() - start_time
            print_results(iteration, elapsed_time)
            # store_stats(iteration, elapsed_time)
            return

if __name__ == "__main__":
    main()