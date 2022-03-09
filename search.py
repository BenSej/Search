# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Kelvin Ma (kelvinm2@illinois.edu) on 01/24/2021

"""
This is the main entry point for MP3. You should only modify code
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
# Note that if you want to test one of your search methods, please make sure to return a blank list
#  for the other search methods otherwise the grader will not crash.

from collections import deque
import heapq


class MST:
    def __init__(self, objectives):
        self.elements = {key: None for key in objectives}

        # TODO: implement some distance between two objectives
        # ... either compute the shortest path between them, or just use the manhattan distance between the objectives
        self.distances = {
                (i, j): self.distance(i, j)
                for i, j in self.cross(objectives)
            }

    def distance(self, i, j):
        return manhattan(i, j)

    # Prim's algorithm adds edges to the MST in sorted order as long as they don't create a cycle
    def compute_mst_weight(self):
        weight = 0
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

def bfs(maze):
    """
    Runs BFS for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    map = {}
    path = []
    explored = []
    queue = deque([])
    start = maze.start
    goal = maze.waypoints[0]
    queue.append(start)
    map[start] = None

    while queue:
        current = queue.popleft()
        if current == goal:
            while map[current] != None:
                path.append(current)
                current = map[current]
            path.append(current)
            path.reverse()
            return path

        explored.append(current)
        for neighbor in maze.neighbors(current[0], current[1]):
            if neighbor not in explored and neighbor not in queue:
                queue.append(neighbor)
                map[neighbor] = current

    return path

def astar_single(maze):
    """
    Runs A star for part 2 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    map = {}
    cost = {}
    queue = []
    explored = []
    start = maze.start
    goal = maze.waypoints[0]
    path = []

    heapq.heappush(queue, (0, start))
    cost[start] = 0
    while queue:
        current = heapq.heappop(queue)
        if current[1] == goal:
            while current[0] != 0:
                path.append(current[1])
                current = map[current[1]]
            path.append(current[1])
            path.reverse()
            return path
        explored.append(current[1])
        for neighbor in maze.neighbors(current[1][0], current[1][1]):
            if neighbor not in explored and neighbor not in queue:
                manhattanDistance = manhattan(neighbor, goal)
                c = cost[current[1]] + 1
                cost[neighbor] = c
                distance = manhattanDistance + c
                explored.append(neighbor)
                heapq.heappush(queue, (distance, neighbor))
                map[neighbor] = current

    return path


def manhattan(x, y):
    return abs(x[0] - y[0]) + abs(x[1] - y[1])


def astar_multiple(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """

    mst = {}
    cost = {}
    queue = []
    explored = {}
    start = maze.start
    waypoints = maze.waypoints
    path = []

    startState = (0, waypoints, start)
    s = (waypoints, start)
    explored[s] = 0
    cost[startState] = 0
    heapq.heappush(queue, startState)

    while queue:
        #for node in queue:
            #print(node[0], len(node[1]), node[2])
        current = heapq.heappop(queue)
        #print("current: " + str(current))

        temp = (current[1], current[2])
        tempC = cost[current]
        if current[2] in current[1]:
            goalsLeft = list(current[1])
            goalsLeft.remove(current[2])
            current = (current[0], tuple(goalsLeft), current[2])
            c = (current[1], current[2])
            explored[c] = temp
            cost[current] = tempC

        if len(current[1]) == 0:
            path.append(current[2])
            c = (current[1], current[2])
            while explored[c] is not 0:
                c = explored[c]
                if c[1] != path[-1]:
                    path.append(c[1])
            path.reverse()
            print(path)
            return path

        for neighbor in maze.neighbors(current[2][0], current[2][1]):
            closestGoalDistance = manhattan(neighbor, current[1][0])
            closestGoal = current[1][0]
            for w in current[1]:
                distance = manhattan(neighbor, w)
                if distance < closestGoalDistance:
                    closestGoal = w
                    closestGoalDistance = distance

            tree = mst.get(current[1], 0)
            if tree == 0:
                tree = MST(current[1])
            mstLength = tree.compute_mst_weight()
            heuristic = closestGoalDistance + mstLength
            #print(closestGoalDistance, mstLength)
            #print("")

            distance = cost[current] + heuristic + 1
            neighborState = (distance, current[1], neighbor)
            n = (neighborState[1], neighborState[2])
            cost[neighborState] = cost[current] + 1

            if n not in explored:
                heapq.heappush(queue, neighborState)
                explored[n] = (current[1], current[2])

    return path


def fast(maze):
    """
    Runs suboptimal search algorithm for extra credit/part 4.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    mst = {}
    cost = {}
    queue = []
    explored = {}
    start = maze.start
    waypoints = maze.waypoints
    path = []

    startState = (0, waypoints, start)
    s = (waypoints, start)
    explored[s] = 0
    cost[startState] = 0
    heapq.heappush(queue, startState)

    while queue:
        current = heapq.heappop(queue)

        temp = (current[1], current[2])
        tempC = cost[current]
        if current[2] in current[1]:
            goalsLeft = list(current[1])
            goalsLeft.remove(current[2])
            current = (current[0], tuple(goalsLeft), current[2])
            c = (current[1], current[2])
            explored[c] = temp
            cost[current] = tempC

        if len(current[1]) == 0:
            path.append(current[2])
            c = (current[1], current[2])
            while explored[c] is not 0:
                c = explored[c]
                if c[1] != path[-1]:
                    path.append(c[1])
            path.reverse()
            print(path)
            return path

        for neighbor in maze.neighbors(current[2][0], current[2][1]):
            closestGoalDistance = manhattan(neighbor, current[1][0])
            for w in current[1]:
                distance = manhattan(neighbor, w)
                if distance < closestGoalDistance:
                    closestGoalDistance = distance

            tree = mst.get(current[1], 0)
            if tree == 0:
                tree = MST(current[1])
            mstLength = tree.compute_mst_weight()
            heuristic = closestGoalDistance + mstLength
            heuristic *= 1.5

            distance = cost[current] + heuristic + 1
            neighborState = (distance, current[1], neighbor)
            n = (neighborState[1], neighborState[2])
            cost[neighborState] = cost[current] + 1

            if n not in explored:
                heapq.heappush(queue, neighborState)
                explored[n] = (current[1], current[2])

    return path
