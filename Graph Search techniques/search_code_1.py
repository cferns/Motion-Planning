#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
import numpy as np
import heapq
from collections import deque
import matplotlib.pyplot as plotter
import math
from math import hypot

#initializing some variables
_DEBUG = False
_DEBUG_END = True
_ACTIONS = ['u','d','l','r']
_ACTIONS_2 = ['u','d','l','r','ne','nw','sw','se']
_X = 1  # we're taking the columns as x and rows as y, so now the axes transform, hence this (x=1 and y=0) and not (x=0 and y=1)
_Y = 0
_GOAL_COLOR = 0.75
_INIT_COLOR = 0.25
_PATH_COLOR_RANGE = _GOAL_COLOR-_INIT_COLOR
_VISITED_COLOR = 0.9

class GridMap:
    '''
    Class to hold a grid map for navigation. Reads in a map.txt file of the format
    0 - free cell, x - occupied cell, g - goal location, i - initial location.
    Additionally provides a simple transition model for grid maps and a convience function
    for displaying maps.
    '''

    def __init__(self, map_path=None):
        '''
        Constructor: Makes the necessary class variables.
        Optionally reads in a provided map file given by map_path.
        map_path (optional) - a string of the path to the file on disk
        '''
        self.rows = None
        self.cols = None
        self.goal = None
        self.init_pos = None
        self.occupancy_grid = None
        if map_path is not None:
            self.read_map(map_path)

    def read_map(self, map_path):
        '''
        Read in a specified map file of the format described in the class doc string.
        map_path - a string of the path to the file on disk
        '''
        map_file = file(map_path,'r')
        lines = [l.rstrip().lower() for l in map_file.readlines()]
                #lines=[]  will create a list of lines
                #str.rstrip() will delete any whitespaces at the end of the line
                #str.lower() will convert all characters into lowercase characters
        map_file.close()

        self.rows = len(lines)                       #rows= number of lines
        self.cols = max([len(l) for l in lines])     #finds max length of each line
        if _DEBUG:
            print 'rows:', self.rows
            print 'cols:', self.cols
            print lines
        self.occupancy_grid = np.zeros((self.rows, self.cols), dtype=np.bool) #is this np.bool_
                        #numpy is defined as np
                        #the above syntax is: numpy.zeros(shape, dtype=float, order='C')
                        #it Returns a new array of given shape and type, filled with zeros.
        for r in xrange(self.rows):
            for c in xrange(self.cols):
                if lines[r][c] == 'x':
                    self.occupancy_grid[r][c] = True        #x is occupied cell
                if lines[r][c] == 'g':
                    self.goal = (r,c)                       #ok, the goal is r c
                elif lines[r][c] == 'i':
                    self.init_pos = (r,c)                   #init is r c

    def is_goal(self,s):
        '''
        to Test if a specified state is the goal state

        s - tuple describing the state as (row, col) position on the grid.

        Returns - True if s is the goal. False otherwise.
        '''
        return (s[_X] == self.goal[_X] and
                s[_Y] == self.goal[_Y])      #is'nt tuple given as () so why s[]

    def transition(self, s, a):
        '''
        Transition function for the current grid map.

        s - tuple describing the state as (row, col) position on the grid.
        a - the action to be performed from state s

        returns - s_prime, the state transitioned to by taking action a in state s.
        If the action is not valid (e.g. moves off the grid or into an obstacle)
        then it returns the current state.
        '''

        new_pos = list(s[:])
        # Ensure action stays on the board
        if a == 'r':
            if s[_Y] > 0:
                new_pos[_Y] -= 1
        elif a == 'l':
            if s[_Y] < self.rows - 1:
                new_pos[_Y] += 1
        elif a == 'd':
            if s[_X] > 0:
                new_pos[_X] -= 1
        elif a == 'u':
            if s[_X] < self.cols - 1:
                new_pos[_X] += 1
        elif a == 'ne':
            if ((s[_Y] > 0) and (s[_X] < self.cols - 1)):
                new_pos[_Y] -= 1
                new_pos[_X] += 1
        elif a == 'sw':
            if ((s[_Y] < self.rows - 1) and (s[_X] > 0)):
                new_pos[_Y] += 1
                new_pos[_X] -= 1
        elif a == 'nw':
            if ((s[_X] > 0) and (s[_Y] > 0)):
                new_pos[_X] -= 1
                new_pos[_Y] -= 1
        elif a == 'se':
            if ((s[_X] < self.cols - 1) and (s[_Y] < self.rows - 1)):
                new_pos[_X] += 1
                new_pos[_Y] += 1
        else:
            print 'Unknown action:', str(a)

        # Test if new position is clear
        if self.occupancy_grid[new_pos[0], new_pos[1]]:
            s_prime = tuple(s)
        else:
            s_prime = tuple(new_pos)
        return s_prime

    def display_map(self, path=[], visited={}):
        '''
        Visualize the map read in. Optionally display the resulting plan and visited nodes

        path - a list of tuples describing the path take from init to goal
        visited - a set of tuples describing the states visited during a search
        '''
        display_grid = np.array(self.occupancy_grid, dtype=np.float32)

        # Color all visited nodes if requested
        for v in visited:
            display_grid[v] = _VISITED_COLOR
        # Color path in increasing color from init to goal
        for i, p in enumerate(path):
            disp_col = _INIT_COLOR + _PATH_COLOR_RANGE*(i+1)/len(path)
            display_grid[p] = disp_col

        display_grid[self.init_pos] = _INIT_COLOR
        display_grid[self.goal] = _GOAL_COLOR

        # Plot display grid for visualization
        imgplot = plotter.imshow(display_grid)
        # Set interpolation to nearest to create sharp boundaries
        imgplot.set_interpolation('nearest')
        # Set color map to diverging style for contrast
        imgplot.set_cmap('spectral')
        plotter.show()

def uninformed_heuristic(s, goal_state):
        '''
        Example of how a heuristic may be provided. This one is admissable, but dumb.

        s - tuple describing the state as (row, col) position on the grid.

        returns - floating point estimate of the cost to the goal from state s
        '''
        deltaX = abs(goal_state[0] - s[0])
        deltaY = abs(goal_state[1] - s[1])
        #distance= deltaX+deltaY          #for manhattan use this
        distance= hypot(deltaX,deltaY)  #for euclidean use this
        return distance

class SearchNode:
    def __init__(self, s, A, parent=None, parent_action=None , costvalue =0 ):
        '''
        s - the state defining the search node
        A - list of actions
        parent - the parent search node
        parent_action - the action taken from parent to get to s
        '''
        self.parent = parent
        self.cost = costvalue
        self.parent_action = parent_action
        self.state = s[:]
        self.actions = A[:]

    def __str__(self):
        '''
        Return a human readable description of the node
        '''
        return str(self.state) + ' ' + str(self.actions)+' '+str(self.parent)+' '+str(self.parent_action)

class PriorityQ:
    '''
    Priority queue implementation with quick access for membership testing
    Setup currently to only with the SearchNode class
    '''
    def __init__(self):
        '''
        Initialize an empty priority queue
        '''
        self.l = [] # list storing the priority q
        self.s = set() # set for fast membership testing

    def __contains__(self, x):
        '''
        Test if x is in the queue
        '''
        return x in self.s

    def push(self, x, cost):
        '''
        Adds an element to the priority queue.
        If the state already exists, we update the cost
        '''
        if x.state in self.s:
            return self.replace(x, cost)
                #heapq.heappush(heap, item)
                #Push the value item onto the heap, maintaining the heap invariant.
        heapq.heappush(self.l, (cost, x))
        self.s.add(x.state)

    def pop(self):
        '''
        Get the value and remove the lowest cost element from the queue
        '''
        x = heapq.heappop(self.l)
        self.s.remove(x[1].state)
        return x[1]

    def peak(self):
        '''
        Get the value of the lowest cost element in the priority queue
        '''
        x = self.l[0]
        return x[1]

    def __len__(self):
        '''
        Return the number of elements in the queue
        '''
        return len(self.l)

    def replace(self, x, new_cost):
        '''
        Removes element x from the q and replaces it with x with the new_cost
        '''
        for y in self.l:
            if x.state == y[1].state:
                self.l.remove(y)
                self.s.remove(y[1].state)
                break
        heapq.heapify(self.l)
        self.push(x, new_cost)

    def get_cost(self, x):
        '''
        Return the cost for the search node with state x.state
        '''
        for y in self.l:
            if x.state == y[1].state:
                return y[0]

    def __str__(self):
        '''
        Return a string of the contents of the list
        '''
        return str(self.l)

def dfs(init_state, f, is_goal, actions, goal_state):
    '''
    Perform depth first search on a grid map.

    init_state - the intial state on the map
    f - transition function of the form s_prime = f(s,a)
    is_goal - function taking as input a state s and returning True if its a goal state
    actions - set of actions which can be taken by the agent

    returns - ((path, action_path), visited) or None if no path can be found
    path - a list of tuples. The first element is the initial state followed by all states
        traversed until the final goal state
    action_path - the actions taken to transition from the initial state to goal state
    '''
    # Fill me in!

    frontier = [] # Search stack
    n0 = SearchNode(init_state, actions)
    visited = []
    frontier.append(n0)
    loop_count = 0
    while len(frontier) > 0:
        # Peak last element
        n_i = frontier.pop()
        if n_i.state not in visited:
            visited.append(n_i.state)
            if is_goal(n_i.state):
                return (backpath(n_i, init_state ), visited)
            else:
                for a in actions:
                    s_prime = f(n_i.state, a)
                    n_prime = SearchNode(s_prime, actions, n_i, a)
                    frontier.append(n_prime)
    return None

def bfs(init_state, f, is_goal, actions, goal_state):
    '''
    Perform breadth first search on a grid map.

    init_state - the intial state on the map
    f - transition function of the form s_prime = f(s,a)
    is_goal - function taking as input a state s and returning True if its a goal state
    actions - set of actions which can be taken by the agent

    returns - ((path, action_path), visited) or None if no path can be found
    path - a list of tuples. The first element is the initial state followed by all states
        traversed until the final goal state
    action_path - the actions taken to transition from the initial state to goal state
    '''
    # Fill me in!

    frontier = [] # Search stack
    #frontier = deque([]) # Search stack   #also can be used
    n0 = SearchNode(init_state, actions)
    visited = []
    frontier.append(n0)
    loop_count = 0
    while len(frontier) > 0:
        # Peak last element
        n_i = frontier.pop(0)
        #n_i = frontier.popleft()   #this also can be used
        if n_i.state not in visited:
            visited.append(n_i.state)
            if is_goal(n_i.state):
                return (backpath(n_i, init_state ), visited)
            else:
                for a in actions:
                    s_prime = f(n_i.state, a)
                    n_prime = SearchNode(s_prime, actions, n_i, a)
                    frontier.append(n_prime)
    return None

def uniform_cost_search(init_state, f, is_goal, actions, goal_state):

    frontier = PriorityQ()                 # Search queue is an object of PriorityQ
    n0 = SearchNode(init_state, actions)
    visited = []
    frontier.push(n0,n0.cost)
    loop_count = 0
    while len(frontier) > 0:
        # Peak last element
        n_i = frontier.pop()
        if n_i.state not in visited:
            visited.append(n_i.state)
            if is_goal(n_i.state):
                return (backpath(n_i, init_state ), visited)
            else:
                for a in actions:
                    s_prime = f(n_i.state, a)
                    if(s_prime != n_i.state):
                        cost = n_i.cost + 1
                        n_prime = SearchNode(s_prime, actions, n_i, a, cost)
                        frontier.push(n_prime,n_prime.cost)
    return None

def a_star_search_manhattan(init_state, f, is_goal, actions, goal_state):

    '''
    init_state - value of the initial state
    f - transition function takes input state (s), action (a), returns s_prime = f(s, a)
        returns s if action is not valid
    is_goal - takes state as input returns true if it is a goal state
        actions - list of actions available
    h - heuristic function, takes input s and returns estimated cost to goal
        (note h will also need access to the map, so should be a member function of GridMap)
    '''

    visited = []
    frontier = PriorityQ()
    cost=0
    n0 = SearchNode(init_state, actions)
    frontier.push(n0,cost)
    loop_count = 0
    while len(frontier) > 0:
        n_i = frontier.pop()
        if n_i.state not in visited:
            visited.append(n_i.state)
            if is_goal(n_i.state):
                return (backpath(n_i, init_state ), visited)
            else:
                for a in actions:
                    s_prime = f(n_i.state, a)
                    if(s_prime != n_i.state):
                        if((a == 'u') or (a == 'd') or (a == 'l') or (a == 'r')):
                            g_value = n_i.cost + 1.0
                        else:
                            g_value = n_i.cost + 1.5
                        deltaX = abs(goal_state[0] - s_prime[0])
                        deltaY = abs(goal_state[1] - s_prime[1])
                        h_value= deltaX+deltaY          #for manhattan use this
                        #h_value= hypot(deltaX,deltaY)    #for euclidean use this
                        #h_value = uninformed_heuristic(s_prime, goal_state)
                        f_value=g_value+h_value
                        n_prime = SearchNode(s_prime, actions, n_i, a, f_value)
                        frontier.push(n_prime,f_value)
    return None

def a_star_search_euclidean(init_state, f, is_goal, actions, goal_state):

    '''
    init_state - value of the initial state
    f - transition function takes input state (s), action (a), returns s_prime = f(s, a)
        returns s if action is not valid
    is_goal - takes state as input returns true if it is a goal state
        actions - list of actions available
    h - heuristic function, takes input s and returns estimated cost to goal
        (note h will also need access to the map, so should be a member function of GridMap)
    '''

    visited = []
    frontier = PriorityQ()
    cost=0
    n0 = SearchNode(init_state, actions)
    frontier.push(n0,cost)
    loop_count = 0
    while len(frontier) > 0:
        n_i = frontier.pop()
        if n_i.state not in visited:
            visited.append(n_i.state)
            if is_goal(n_i.state):
                return (backpath(n_i, init_state ), visited)
            else:
                for a in actions:
                    s_prime = f(n_i.state, a)
                    if(s_prime != n_i.state):
                        if((a == 'u') or (a == 'd') or (a == 'l') or (a == 'r')):
                            g_value = n_i.cost + 1.0
                        else:
                            g_value = n_i.cost + 1.5
                        deltaX = abs(goal_state[0] - s_prime[0])
                        deltaY = abs(goal_state[1] - s_prime[1])
                        #h_value= deltaX+deltaY          #for manhattan use this
                        h_value= hypot(deltaX,deltaY)    #for euclidean use this
                        #h_value = uninformed_heuristic(s_prime, goal_state)
                        f_value=g_value+h_value
                        n_prime = SearchNode(s_prime, actions, n_i, a, f_value)
                        frontier.push(n_prime,f_value)
    return None

def backpath(node, initial_position) :
    '''
    Function to determine the path that lead to the specified search node
    node - the SearchNode that is the end of the path
    returns - a tuple containing (path, action_path) which are lists respectively of the states
    visited from init to goal (inclusive) and the actions taken to make those transitions.
    '''

    back_path = []
    path = []
    back_action_path = []
    n0 = node
    back_path.append(n0.state)
    back_action_path.append(n0.parent_action)
    while(n0.state != initial_position):
        back_path.append(n0.parent.state)
        back_action_path.append(n0.parent.parent_action)
        n0 = n0.parent
    path = back_path[::-1]  #reverse list contents
    action_path = back_action_path[::-1]  #reverse list contents
    return(path, action_path)


print "Choose any one Combination of algorithms and maps:"
print "1: dfs and map0"
print "2: dfs and map1"
print "3: dfs and map2"
print "4: bfs and map0"
print "5: bfs and map1"
print "6: bfs and map2"
print "7: uniform_search and map0"
print "8: uniform_search and map1"
print "9: uniform_search and map2"
print "10: a_star_search with manhattan heuristic and map0"
print "11: a_star_search with manhattan heuristic and map1"
print "12: a_star_search with manhattan heuristic and map2"
print "13: a_star_search with euclidean heuristic and map0"
print "14: a_star_search with euclidean heuristic and map1"
print "15: a_star_search with euclidean heuristic and map2"
choice = input("ENTER CHOICE NUMBER: ")

print "Choose the type of actions required:"
print "1: u, d, l, r"
print "2: u, d, l, r, ne, nw, se, sw"
actChoice = input("ENTER CHOICE NUMBER: ")

if(actChoice==1):
    act=_ACTIONS
elif(actChoice==2):
    act=_ACTIONS_2

if (choice==1):
    g= GridMap('./map0.txt')
    g.display_map()
    [[path,action_path],visited] = dfs(g.init_pos, g.transition, g.is_goal, act, g.goal)
    print "visited: ", visited
    print "path: ", path
    print "action_path: ", action_path
    g.display_map(path, visited)

elif (choice==2):
    g= GridMap('./map1.txt')
    g.display_map()
    [[path,action_path],visited] = dfs(g.init_pos, g.transition, g.is_goal, act, g.goal)
    print "visited: ", visited
    print "path: ", path
    print "action_path: ", action_path
    g.display_map(path, visited)

elif (choice==3):
    g= GridMap('./map2.txt')
    g.display_map()
    [[path,action_path],visited] = dfs(g.init_pos, g.transition, g.is_goal, act, g.goal)
    print "visited: ", visited
    print "path: ", path
    print "action_path: ", action_path
    g.display_map(path, visited)

elif (choice==4):
    g= GridMap('./map0.txt')
    g.display_map()
    [[path,action_path],visited] = bfs(g.init_pos, g.transition, g.is_goal, act, g.goal)
    print "visited: ", visited
    print "path: ", path
    print "action_path: ", action_path
    g.display_map(path, visited)

elif (choice==5):
    g= GridMap('./map1.txt')
    g.display_map()
    [[path,action_path],visited] = bfs(g.init_pos, g.transition, g.is_goal, act, g.goal)
    print "visited: ", visited
    print "path: ", path
    print "action_path: ", action_path
    g.display_map(path, visited)

elif (choice==6):
    g= GridMap('./map2.txt')
    g.display_map()
    [[path,action_path],visited] = bfs(g.init_pos, g.transition, g.is_goal, act, g.goal)
    print "visited: ", visited
    print "path: ", path
    print "action_path: ", action_path
    g.display_map(path, visited)
    
elif (choice==7):
    g= GridMap('./map0.txt')
    g.display_map()
    [[path,action_path],visited] = uniform_cost_search(g.init_pos, g.transition, g.is_goal, act, g.goal)
    print "visited: ", visited
    print "path: ", path
    print "action_path: ", action_path
    g.display_map(path, visited)

elif (choice==8):
    g= GridMap('./map1.txt')
    g.display_map()
    [[path,action_path],visited] = uniform_cost_search(g.init_pos, g.transition, g.is_goal, act, g.goal)
    print "visited: ", visited
    print "path: ", path
    print "action_path: ", action_path
    g.display_map(path, visited)

elif (choice==9):
    g= GridMap('./map2.txt')
    g.display_map()
    [[path,action_path],visited] = uniform_cost_search(g.init_pos, g.transition, g.is_goal, act, g.goal)
    print "visited: ", visited
    print "path: ", path
    print "action_path: ", action_path
    g.display_map(path, visited)
    
elif (choice==10):
    g= GridMap('./map0.txt')
    g.display_map()
    [[path,action_path],visited] = a_star_search_manhattan(g.init_pos, g.transition, g.is_goal, act, g.goal)
    print "visited: ", visited
    print "path: ", path
    print "action_path: ", action_path
    g.display_map(path, visited)

elif (choice==11):
    g= GridMap('./map1.txt')
    g.display_map()
    [[path,action_path],visited] = a_star_search_manhattan(g.init_pos, g.transition, g.is_goal, act, g.goal)
    print "visited: ", visited
    print "path: ", path
    print "action_path: ", action_path
    g.display_map(path, visited)

elif (choice==12):
    g= GridMap('./map2.txt')
    g.display_map()
    [[path,action_path],visited] = a_star_search_manhattan(g.init_pos, g.transition, g.is_goal, act, g.goal)
    print "visited: ", visited
    print "path: ", path
    print "action_path: ", action_path
    g.display_map(path, visited)
    
    
elif (choice==13):
    g= GridMap('./map0.txt')
    g.display_map()
    [[path,action_path],visited] = a_star_search_euclidean(g.init_pos, g.transition, g.is_goal, act, g.goal)
    print "visited: ", visited
    print "path: ", path
    print "action_path: ", action_path
    g.display_map(path, visited)

elif (choice==14):
    g= GridMap('./map1.txt')
    g.display_map()
    [[path,action_path],visited] = a_star_search_euclidean(g.init_pos, g.transition, g.is_goal, act, g.goal)
    print "visited: ", visited
    print "path: ", path
    print "action_path: ", action_path
    g.display_map(path, visited)

elif (choice==15):
    g= GridMap('./map2.txt')
    g.display_map()
    [[path,action_path],visited] = a_star_search_euclidean(g.init_pos, g.transition, g.is_goal, act, g.goal)
    print "visited: ", visited
    print "path: ", path
    print "action_path: ", action_path
    g.display_map(path, visited)
    
else:
    print "Invalid Entry, run the code again"