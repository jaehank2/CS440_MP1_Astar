# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Kelvin Ma (kelvinm2@illinois.edu) on 01/24/2021

"""
This is the main entry point for MP1. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""
# Search should return the path.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi,fast)


# Feel free to use the code below as you wish
# Initialize it with a list/tuple of objectives
# Call compute_mst_weight to get the weight of the MST with those objectives
# TODO: hint, you probably want to cache the MST value for sets of objectives you've already computed...

import heapq
import copy


class MST:
    def __init__(self, objectives):
        self.elements = {key: None for key in objectives}

        # TODO: implement some distance between two objectives 
        # ... either compute the shortest path between them, or just use the manhattan distance between the objectives
        #print("INIT")
        # for i, j in self.cross(objectives):
        #     print(i, j)
        self.distances   = {
                # (i, j): DISTANCE(i, j)
                (i, j): abs(i[0] - j[0]) + abs(i[1] - j[1])
                for i, j in self.cross(objectives)
            }
        # print("***")
        # print(self.distances[(1, 1), (1, 35)])
        
    # Prim's algorithm adds edges to the MST in sorted order as long as they don't create a cycle
    def compute_mst_weight(self):
        weight      = 0
        for distance, i, j in sorted((self.distances[(i, j)], i, j) for (i, j) in self.distances):
            if self.unify(i, j):
                weight += distance
        return weight

    # helper checks the root of a node, in the process flatten the path to the root
    def resolve(self, key):
        path = []
        root = key 
        while self.elements[root] is not None:
            path.append(root)
            root = self.elements[root]
        for key in path:
            self.elements[key] = root
        return root
    
    # helper checks if the two elements have the same root they are part of the same tree
    # otherwise set the root of one to the other, connecting the trees
    def unify(self, a, b):
        ra = self.resolve(a) 
        rb = self.resolve(b)
        if ra == rb:
            return False 
        else:
            self.elements[rb] = ra
            return True

    # helper that gets all pairs i,j for a list of keys
    def cross(self, keys):
        return (x for y in (((i, j) for j in keys if i < j) for i in keys) for x in y)

# bfs with single waypoint
def bfs(maze):
    """
    Runs BFS for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # starting coordinate as tuple (x,y)
    s = maze.start

    # ending coordinate as tuple (x, y)
    goal = maze.waypoints[0]
    print(s, goal)

    # 2D list to keep track of visited states
    visited = [[False for j in range(maze.size.x)] for i in range(maze.size.y)]

    # list (queue) to keep track of todo states
    todo = [s]

    # dictionary to keep track of parent nodes. (parent[x] = y denotes y is x's parent) - might need to find a better way
    parent = {}

    # returned path from start to goal
    answer = [goal]

    # traverse until goal is found through bfs
    while todo:
        cur = todo.pop(0)
        if cur == goal:
            break
        if visited[cur[0]][cur[1]] is False:
            visited[cur[0]][cur[1]] = True
            nbrs = maze.neighbors(cur[0], cur[1])
            for n in nbrs:
                if visited[n[0]][n[1]] is False:
                    parent[n] = cur
            todo.extend(nbrs)
    
    temp = goal
    
    # add parent states to retrieve path
    while temp != s:
        answer.insert(0, parent[temp])
        temp = parent[temp]

    return answer


# astar with single waypoint
def astar_single(maze):
    """
    Runs A star for part 2 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # Three types of states for A*
    # 1. unseen states
    # 2. done/closed states = neighbors explored
    # 3. open/frontier states = seen but neighbors not explored
    #
    # Will be using Manhattan distance for the heuristic function
    # cost function is defined as:
    #   f_cost = g_cost + h_cost
    # where g_cost is the distance from start and h_cost is the heuristic cost


    # starting coordinate
    s = maze.start

    # ending coordinate
    goal = maze.waypoints[0]
    print(s, goal)

    # 2D list to keep track of min g_cost of each state. Initialized to inf
    visited = [[float('inf') for j in range(maze.size.x)] for i in range(maze.size.y)]

    # cost for s: manhattan heuristic
    cost = abs(s[0] - goal[0]) + abs(s[1] - goal[1])

    # initalize priority queue for open states
    open = []
    heapq.heappush(open, (cost, s))

    # dictionary to keep track of parent nodes. (parent[x] = y denotes y is x's parent) - used to retrieve back the path
    parent = {}

    # returned path from start to goal
    answer = [goal]

    while open:
        # current state with least cost
        f_cost, state = heapq.heappop(open)
        h_cost = abs(state[0] - goal[0]) + abs(state[1] - goal[1])
        g_cost = f_cost - h_cost
        if state == goal:
            break
        # update current state's values in closed set
        nbrs = maze.neighbors(state[0], state[1])
        g_cost += 1
        for n in nbrs:
            if g_cost < visited[n[0]][n[1]]:
                visited[n[0]][n[1]] = g_cost
                parent[n] = state
                h_cost = abs(n[0] - goal[0]) + abs(n[1] - goal[1])
                f_cost = g_cost + h_cost
                heapq.heappush(open, (f_cost, n))


    temp = goal
    
    # add parent states to retrieve path
    while temp != s:
        answer.insert(0, parent[temp])
        temp = parent[temp]


    return answer

def nearest_waypt_dist(waypoints, coord):
    dist = float('inf')

    for waypt in waypoints:
        temp = abs(coord[0] - waypt[0]) + abs(coord[1] - waypt[1])
        if temp < dist:
            dist = temp
    
    return dist

# astar with multiple waypoints
def astar_multiple(maze):

    """
    Runs A star for part 3 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """

    # list of waypoints to traverse
    goals = list(maze.waypoints)
    print(goals)

    # states represented as a tuple of (x,y) + list of remaining waypoints left
    s = (maze.start, goals)

    # dictionary of dictionaries to keep track of parent nodes.
    # (parent[x][y] = w, z denotes parent of a state with "x" coord and "y" remaining waypts is a state with "w" coord and "z" remaining waypts
    parent = {}

    # dictionary to cache MST values to save computation time
    cache = {}

    # returned path from start to goal
    answer = []


    myMST = MST(goals)
    h_cost = myMST.compute_mst_weight()
    #dist = nearest_waypt_dist(goals, s[0])

    # cache mst weight for case of "no waypoints visited"
    key = tuple(goals)
    cache[key] = h_cost

    g_cost = 0
    f_cost = g_cost + h_cost #+ dist

    # initalize priority queue for open states
    open = []
    heapq.heappush(open, (f_cost, s))

    # dictionary of dictionaries to keep track of min f_cost of each state: (x,y) -> remaining goals -> f_cost
    best_cost = {s[0] : {tuple(goals) : f_cost}}

    # used to backtrace from goal to start to retrieve path. Initialized to start state
    backtrace_xy = s[0]
    backtrace_todo = tuple(goals)

    while open:
        # xy = coordinate and todo = list of remaining waypoints left
        f_cost, cur_state = heapq.heappop(open)
        xy = cur_state[0]
        todo = copy.copy(cur_state[1])
        waypoints = tuple(todo)
        cur_g_cost = f_cost - cache[waypoints] #- nearest_waypt_dist(todo, xy)

        # if no more waypoints left, break
        if len(todo) == 0:
            backtrace_xy = xy
            backtrace_todo = tuple(todo)
            answer.append(xy)
            break
        
        nbrs = maze.neighbors(xy[0], xy[1])
        #g_cost += 1
        g_cost = cur_g_cost + 1
        for n in nbrs:
            # list of remaining waypoints
            todo = copy.copy(cur_state[1])
            # check if neighbor is one of waypoints. If so, remove the waypoint
            if n in todo:
                todo.remove(n)
            waypoints = tuple(todo)
            # calculate cost
            if waypoints not in cache:
                myMST = MST(todo)
                h_cost = myMST.compute_mst_weight()
                cache[waypoints] = h_cost
            else:
                h_cost = cache[waypoints]
            #dist = nearest_waypt_dist(todo, n)
            f_cost = g_cost + h_cost #+ dist

            # if neighbor visited and current f_cost is higher, skip
            if n in best_cost and waypoints in best_cost[n] and f_cost >= best_cost[n][waypoints]:
                continue
            # case where n isn't a key (not visited)
            elif n not in best_cost:
                best_cost[n] = {}
                best_cost[n][waypoints] = f_cost
            # case where n is a key, but waypoints isn't OR g_cost is lower
            elif waypoints not in best_cost[n] or f_cost < best_cost[n][waypoints]:
                best_cost[n][waypoints] = f_cost
            
            # update parent
            if n in parent:
                parent[n][waypoints] = (xy, cur_state[1])
            else:
                parent[n] = {}
                parent[n][waypoints] = (xy, cur_state[1])
            heapq.heappush(open, (f_cost, (n, todo)))
    
    while (backtrace_xy != s[0]) or (backtrace_todo != tuple(goals)):
        coord, waypts = parent[backtrace_xy][backtrace_todo]
        answer.insert(0, coord)
        backtrace_xy = coord
        backtrace_todo = tuple(waypts)


    return answer

def fast(maze):
    """
    Runs suboptimal search algorithm for part 4.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """

    # list of waypoints to traverse
    goals = list(maze.waypoints)
    print(goals)

    # states represented as a tuple of (x,y) + list of remaining waypoints left
    s = (maze.start, goals)

    # dictionary of dictionaries to keep track of parent nodes.
    # (parent[x][y] = w, z denotes parent of a state with "x" coord and "y" remaining waypts is a state with "w" coord and "z" remaining waypts
    parent = {}

    # dictionary to cache MST values to save computation time
    cache = {}

    # returned path from start to goal
    answer = []

    weight = 2.3
    myMST = MST(goals)
    h_cost = myMST.compute_mst_weight() * weight
    #dist = nearest_waypt_dist(goals, s[0])

    # cache mst weight for case of "no waypoints visited"
    key = tuple(goals)
    cache[key] = h_cost

    g_cost = 0
    f_cost = g_cost + h_cost #+ dist

    # initalize priority queue for open states
    open = []
    heapq.heappush(open, (f_cost, s))

    # dictionary of dictionaries to keep track of min f_cost of each state: (x,y) -> remaining goals -> f_cost
    best_cost = {s[0] : {tuple(goals) : f_cost}}

    # used to backtrace from goal to start to retrieve path. Initialized to start state
    backtrace_xy = s[0]
    backtrace_todo = tuple(goals)

    while open:
        # xy = coordinate and todo = list of remaining waypoints left
        f_cost, cur_state = heapq.heappop(open)
        xy = cur_state[0]
        todo = copy.copy(cur_state[1])
        waypoints = tuple(todo)
        cur_g_cost = f_cost - cache[waypoints] #- nearest_waypt_dist(todo, xy)

        # if no more waypoints left, break
        if len(todo) == 0:
            backtrace_xy = xy
            backtrace_todo = tuple(todo)
            answer.append(xy)
            break
        
        nbrs = maze.neighbors(xy[0], xy[1])
        #g_cost += 1
        g_cost = cur_g_cost + 1
        for n in nbrs:
            # list of remaining waypoints
            todo = copy.copy(cur_state[1])
            # check if neighbor is one of waypoints. If so, remove the waypoint
            if n in todo:
                todo.remove(n)
            waypoints = tuple(todo)
            # calculate cost
            if waypoints not in cache:
                myMST = MST(todo)
                h_cost = myMST.compute_mst_weight() * weight
                cache[waypoints] = h_cost
            else:
                h_cost = cache[waypoints]
            #dist = nearest_waypt_dist(todo, n)
            f_cost = g_cost + h_cost #+ dist

            # if neighbor visited and current f_cost is higher, skip
            if n in best_cost and waypoints in best_cost[n] and f_cost >= best_cost[n][waypoints]:
                continue
            # case where n isn't a key (not visited)
            elif n not in best_cost:
                best_cost[n] = {}
                best_cost[n][waypoints] = f_cost
            # case where n is a key, but waypoints isn't OR g_cost is lower
            elif waypoints not in best_cost[n] or f_cost < best_cost[n][waypoints]:
                best_cost[n][waypoints] = f_cost
            
            # update parent
            if n in parent:
                parent[n][waypoints] = (xy, cur_state[1])
            else:
                parent[n] = {}
                parent[n][waypoints] = (xy, cur_state[1])
            heapq.heappush(open, (f_cost, (n, todo)))
    

    while (backtrace_xy != s[0]) or (backtrace_todo != tuple(goals)):
        coord, waypts = parent[backtrace_xy][backtrace_todo]
        answer.insert(0, coord)
        backtrace_xy = coord
        backtrace_todo = tuple(waypts)


    return answer
    
            
