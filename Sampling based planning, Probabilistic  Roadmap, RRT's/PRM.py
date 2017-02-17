#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
import random
import numpy as np
import matplotlib.pyplot as plotter
from math import pi, sqrt, sin, cos
from collisions import PolygonEnvironment
import time
from math import sqrt,cos,sin,atan2

_DEBUG = False

_TRAPPED = 'trapped'
_ADVANCED = 'advanced'
_REACHED = 'reached'

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
    def __init__(self, s,parent=None,costvalue =0 ):
        '''
        s - the state defining the search node
        A - list of actions
        parent - the parent search node
        parent_action - the action taken from parent to get to s
        '''
        self.parent = parent
        self.cost = costvalue
        self.state = s

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

class TreeNode:
    def __init__(self, state, parent=None):
        self.state = state
        self.children = []          #initiallizing the child list
        self.parent = parent

    def add_child(self, child):
        self.children.append(child)

class RRTSearchTree:                #trying to link the nodes by edges to create a tree
    def __init__(self, init):
        self.root = TreeNode(init)  #takes the root ??? the current state??
        self.nodes = [self.root]
        self.edges = []             #initiallizing the edges list

    def find_nearest(self, s_query):
        min_d = 1000000
        nn = self.root
        for n_i in self.nodes:
            d = np.linalg.norm(s_query - n_i.state)
            if d < min_d:
                nn = n_i
                min_d = d
        return (nn, min_d)

    def add_node(self, node, parent):
        self.nodes.append(node)
        self.edges.append((parent.state, node.state))
        node.parent = parent
        parent.add_child(node)        #assign the current node as the child to the parent??

    def get_states_and_edges(self):
        states = np.array([n.state for n in self.nodes])        #creating a numpy array of nodes/states
        return (states, self.edges)                             #how does this return edges

    def get_back_path(self, n):
        path = []               #list of nodes in back order
        while True:         #condition to run till the first node
            path.append(n.state)
            n = n.parent                    #now consider the previous node
            if n.parent is None:
                path.append(n.state)
                break
        path.reverse()
        return path

    def get_backpath_from_goal(self, n):
        path = []               #list of nodes in back order
        while n.parent is not None:         #condition to run till the first node
            n = n.parent
            path.append(n.state)
        return path

class RRT:

    def __init__(self, num_samples, num_dimensions=2, step_length = 1, lims = None,
                 connect_prob = 0.05, collision_func=None):
        '''
        Initialize an RRT planning instance
        '''
        self.K = num_samples
        self.n = num_dimensions
        self.epsilon = step_length
        self.connect_prob = connect_prob
        self.in_collision = collision_func
        if collision_func is None:
            self.in_collision = self.fake_in_collision              #what is this??

        # Setup range limits
        self.limits = lims
        if self.limits is None:
            self.limits = []
            for n in xrange(num_dimensions):
                self.limits.append([0,100])
            self.limits = np.array(self.limits)              #what is this??

        self.ranges = self.limits[:,1] - self.limits[:,0]
        self.found_path = False

    def build_rrt(self, init, goal):
        self.goal = np.array(goal)
        self.init = np.array(init)
        print "goal", self.goal
        print "start",self.init
        self.found_path = True
        # Build tree and search
        self.T = RRTSearchTree(init)
        connect=False

        for k in range(1,self.K+1):
            q_rand=self.sample(self.limits,k,self.connect_prob,self.goal)
            q_new_says=self.extend(q_rand,connect)
            if(q_new_says=='breakloop'):
                print "k:", k
                break
        (node_list, edge_list)=self.T.get_states_and_edges()
        path=self.T.get_back_path(self.T.nodes[-1])
        return path

    def build_rrt_connect(self, init, goal):
        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False
        # Build tree and search
        self.T=RRTSearchTree(init) #temporary tree
        self.Ta = RRTSearchTree(init)
        self.Tb = RRTSearchTree(goal)
        connect=True
        for k in range(1,self.K+1):
            q_rand=self.sample(self.limits,k,-1,self.goal)
            self.T=self.Ta
            if not (self.extend(q_rand,connect) == 'trapped'):
                self.Ta=self.T
                q_new_target=self.Ta.nodes[-1].state
                self.T=self.Tb
                self.extend(q_new_target,connect)###what to do about the return
                self.Tb=self.T
                qnew_of_other_tree=self.Tb.nodes[-1].state
                if (qnew_of_other_tree==q_new_target).all():
                    print "k:", k # here give the plan
                    if (self.Ta.nodes[0].state==self.init).all():
                        path_a=self.Ta.get_back_path(self.Ta.nodes[-1])
                        path_b=self.Tb.get_backpath_from_goal(self.Tb.nodes[-1])
                        path=path_a+path_b
                        self.T.nodes=self.Ta.nodes+self.Tb.nodes
                        self.T.edges=self.Ta.edges+self.Tb.edges
                        return path
                    else:
                        path_a=self.Tb.get_back_path(self.Tb.nodes[-1])
                        path_b=self.Ta.get_backpath_from_goal(self.Ta.nodes[-1])
                        path=path_a+path_b
                        self.T.nodes=self.Ta.nodes+self.Tb.nodes
                        self.T.edges=self.Ta.edges+self.Tb.edges
                        return path
            #swap Ta and Tb
            Ttemp=self.Ta
            self.Ta=self.Tb
            self.Tb=Ttemp
        print "Failure"
        return None

    def sample(self,limits,k,prob,goal):
        # Return goal with connect_prob probability

        if (random.uniform(0,1)<prob):
            q_rand=goal
        else:
            if(self.n==2):
                x_rand = random.randrange(limits[0][0],limits[0][1])
                y_rand = random.randrange(limits[1][0],limits[1][1])
                q_rand=np.array([x_rand,y_rand])
            elif(self.n==3):
                theta_1 = random.uniform(-pi,pi)
                theta_2 = random.uniform(-pi,pi)
                theta_3 = random.uniform(-pi,pi)
                q_rand=np.array([theta_1,theta_2,theta_3])
        return q_rand

    def extend(self,rand,connect):
        conn=0

        [q_near,dist_min]=self.T.find_nearest(rand)
        (q_new,conn)=self.new_state(q_near.state,rand,self.epsilon,dist_min,connect)
        if(self.in_collision(q_new)):
            return 'trapped'
        else:
            new_node_info = TreeNode(q_new)
            self.T.add_node(new_node_info, q_near)

            while(conn==100):
                d_rand_near = np.linalg.norm(rand - new_node_info.state)
                (q_new,conn)=self.new_state(new_node_info.state,rand,self.epsilon,d_rand_near,connect)
                if(self.in_collision(q_new)):
                    return None
                else:
                    new_node_info = TreeNode(q_new)
                    self.T.add_node(new_node_info, q_near)

            #test goal
            finalReqd=self.goal
            temp_dist = np.linalg.norm(q_new - finalReqd)
            if(temp_dist<self.epsilon):
                q_final=TreeNode(finalReqd)
                self.T.add_node(q_final, self.T.nodes[-1])
                return 'breakloop'

    def new_state(self,q_nearState,rand,epsilon,dist_min,connect):

        if(connect==False):
            dist = np.linalg.norm(rand - q_nearState)
            if dist < epsilon:
                new=rand
                conn=0
                return (np.array(new),conn)
            else:
                new=q_nearState+epsilon*(rand-q_nearState)/(dist_min)
                conn=0
                return (np.array(new),conn)

        else:
            dist = np.linalg.norm(rand - q_nearState)
            if dist < epsilon:
                new=rand
                conn=0
                return (np.array(new),conn)
            else:
                new=q_nearState+epsilon*(rand-q_nearState)/(dist_min)
                conn=100
                return (np.array(new),conn)


    def fake_in_collision(self, q):
        '''
        We never collide with this function!
        '''
        return False

class PRMSearchTree:                #trying to link the nodes by edges to create a tree
    def __init__(self):
        #self.root = TreeNode(init)  #takes the root ??? the current state??
        self.nodes = []
        self.edges = []             #initiallizing the edges list

    def find_nearest(self, s_query):
        min_d = 1000000
        nn = self.root
        for n_i in self.nodes:
            d = np.linalg.norm(s_query - n_i.state)
            if d < min_d:
                nn = n_i
                min_d = d
        return (nn, min_d)

    def find_nearest22(self, s_query,min_d):
        nn = self.nodes[0]
        retRange=[]
        retDist=[]
        for n_i in self.nodes:
            d = np.linalg.norm(s_query.state - n_i.state)
            if d < min_d and d !=0:
                retRange.append(n_i)
                retDist.append(d)
        return retRange,retDist

    def add_node(self, node, parent):
        self.nodes.append(node)
        #self.edges.append((parent.state, node.state))
        node.parent = parent
        #parent.add_child(node)        #assign the current node as the child to the parent??

    def add_edge(self,node,parent):
        self.edges.append((parent.state, node.state))


    def get_states_and_edges(self):
        states = np.array([n.state for n in self.nodes])        #creating a numpy array of nodes/states
        return (states, self.edges)                             #how does this return edges

    def get_back_path(self, n):
        path = []               #list of nodes in back order
        while True:         #condition to run till the first node
            path.append(n.state)
            n = n.parent                    #now consider the previous node
            if n.parent is None:
                path.append(n.state)
                break
        path.reverse()
        return path

    def get_backpath_from_goal(self, n):
        path = []               #list of nodes in back order
        while n.parent is not None:         #condition to run till the first node
            n = n.parent
            path.append(n.state)
        return path

class PRM:

    def __init__(self, num_samples, num_dimensions=2, min_d=5,step_length = 1, lims = None,
                 collision_func=None):
        '''
        Initialize an RRT planning instance
        '''
        self.K = num_samples
        self.n = num_dimensions
        self.min_dist=min_d
        self.epsilon = step_length
        self.in_collision = collision_func
        if collision_func is None:
            self.in_collision = self.fake_in_collision              #what is this??

        # Setup range limits
        self.limits = lims
        if self.limits is None:
            self.limits = []
            for n in xrange(num_dimensions):
                self.limits.append([0,100])
            self.limits = np.array(self.limits)              #what is this??

        self.ranges = self.limits[:,1] - self.limits[:,0]
        self.found_path = False

    def build_prm(self,init,goal):
        self.goal=np.array(goal)
        self.init=np.array(init)
        self.found_path = False
        self.T=PRMSearchTree()
        connect = False

        for k in range(1,self.K+1):
            q_rand=self.sample(self.limits)
            if(self.in_collision(q_rand))==False:
                new_node_info = TreeNode(q_rand)
                self.T.add_node(new_node_info, None)

        for n_i in self.T.nodes:
            (nearPoints,nearDist)=self.T.find_nearest22(n_i,self.min_dist)
            for x in range (0,len(nearPoints)):
                planner_says=self.incrementPlanner(n_i.state,nearPoints[x].state,self.epsilon)
                if planner_says=='pass':
                    self.T.add_edge(nearPoints[x],n_i)

        goalinfo=TreeNode(self.goal)
        for r in self.T.nodes:
            planner2_says=self.incrementPlanner(goalinfo.state,r.state,self.epsilon)
            min_dist=10000
            if planner2_says=='pass':
                dist=np.linalg.norm(goalinfo.state-r.state)
                if dist<min_dist:
                    min_dist=dist
                    nearestToG=r
        self.T.add_edge(nearestToG,goalinfo)

        initinfo=TreeNode(self.init)
        for r in self.T.nodes:
            planner2_says=self.incrementPlanner(initinfo.state,r.state,self.epsilon)
            min_dist=10000
            if planner2_says=='pass':
                dist=np.linalg.norm(initinfo.state-r.state)
                if dist<min_dist:
                    min_dist=dist
                    nearestToI=r
        self.T.add_edge(nearestToI,initinfo)

        #plan = self.a_star_search_euclidean(initinfo,goalinfo,self.T.edges)

        return None

    def sample(self,limits):
        if(self.n==2):
            x_rand = random.randrange(limits[0][0],limits[0][1])
            y_rand = random.randrange(limits[1][0],limits[1][1])
            q_rand=np.array([x_rand,y_rand])
        elif(self.n==3):
            theta_1 = random.uniform(-pi,pi)
            theta_2 = random.uniform(-pi,pi)
            theta_3 = random.uniform(-pi,pi)
            q_rand=np.array([theta_1,theta_2,theta_3])
        return q_rand

    def incrementPlanner(self,current,neighbour,epsilon):
        while(True):
            dist = np.linalg.norm(current - neighbour)
            if dist<epsilon:
                return 'pass'
            new=neighbour+epsilon*(current-neighbour)/(dist)
            neighbour=new
            if(self.in_collision(new)==True):
                return None

    def extend(self,rand,q_near,dist_min,connect):
        conn=0

        #[q_near,dist_min]=self.T.find_nearest(rand)
        (q_new,conn)=self.new_state(q_near.state,rand,self.epsilon,dist_min,connect)
        if(self.in_collision(q_new)):
            return 'trapped'
        else:
            new_node_info = TreeNode(q_new)
            self.T.add_node(new_node_info, q_near)

            while(conn==100):
                d_rand_near = np.linalg.norm(rand - new_node_info.state)
                (q_new,conn)=self.new_state(new_node_info.state,rand,self.epsilon,d_rand_near,connect)
                if(self.in_collision(q_new)):
                    return None
                else:
                    new_node_info = TreeNode(q_new)
                    self.T.add_node(new_node_info, q_near)
                    if q_new==rand:
                        return 'accept'

            #test goal
            finalReqd=self.goal
            temp_dist = np.linalg.norm(q_new - finalReqd)
            if(temp_dist<self.epsilon):
                q_final=TreeNode(finalReqd)
                self.T.add_node(q_final, self.T.nodes[-1])
                return 'breakloop'

    def a_star_search_euclidean(self,init_state, goal_state,edgeList):

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
        n0 = SearchNode(init_state)
        frontier.push(n0,cost)
        loop_count = 0
        while len(frontier) > 0:
            n_i = frontier.pop()
            if n_i.state not in visited:
                visited.append(n_i.state)
                if np.array_equal(n_i.state,goal_state.state):
                    return self.backpath2(n_i, init_state )
                else:
                    for a in edgeList:
                        s_prime = a
                        if(s_prime[0] != n_i.state):
                            g_value = n_i.cost + 1.5
                            h_value= abs(np.linalg.norm(goal_state,s_prime))
                            f_value=g_value+h_value
                            n_prime = SearchNode(s_prime, n_i,f_value)
                            frontier.push(n_prime,f_value)
        return None

    def backpath2(node, initial_position) :
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
        return path

        def new_state(self,q_nearState,rand,epsilon,dist_min,connect):

            if(connect==False):
                dist = np.linalg.norm(rand - q_nearState)
                if dist < epsilon:
                    new=rand
                    conn=0
                    return (np.array(new),conn)
                else:
                    new=q_nearState+epsilon*(rand-q_nearState)/(dist_min)
                    conn=0
                    return (np.array(new),conn)

            else:
                dist = np.linalg.norm(rand - q_nearState)
                if dist < epsilon:
                    new=rand
                    conn=0
                    return (np.array(new),conn)
                else:
                    new=q_nearState+epsilon*(rand-q_nearState)/(dist_min)
                    conn=100
                    return (np.array(new),conn)


        def fake_in_collision(self, q):
            '''
            We never collide with this function!
            '''
            return False

        def fake_in_collision(self, q):
            '''
            We never collide with this function!
            '''
            return False

def test_rrt_env(num_samples=50, step_length=2, env='./env0.txt', connect=True):
    '''
    create an instance of PolygonEnvironment from a description file and plan a path from start to goal on it using an RRT

    num_samples - number of samples to generate in RRT
    step_length - step size for growing in rrt (epsilon)
    env - path to the environment file to read
    connect - If True run rrt_connect

    returns plan, planner - plan is the set of configurations from start to goal, planner is the rrt used for building the plan
    '''
    pe = PolygonEnvironment()
    pe.read_env(env)
    #pe.draw_env(q=(pe.start), show=True)

    dims = len(pe.start)
    start_time = time.time()

#rrt is a class of RRT
    rrt = RRT(num_samples,
              dims,
              step_length,
              lims = pe.lims,
              connect_prob = 0.05,
              collision_func=pe.test_collisions)
    if connect:
        plan = rrt.build_rrt_connect(pe.start, pe.goal)
    else:
        plan = rrt.build_rrt(pe.start, pe.goal)
    run_time = time.time() - start_time
    print 'plan:', plan
    print 'run_time =', run_time
    pe.draw_plan(plan,rrt,True,True)
    plotter.pause(600)
    return plan, rrt

def test_prm_env(num_samples=100, step_length=2,dist_to_nearest_neighbour=40,env='./env0.txt'):

    pe2 = PolygonEnvironment()
    pe2.read_env(env)
    #pe2.draw_env(q=(pe.start), show=True)

    dims = len(pe2.start)
    start_time = time.time()

    prm = PRM(num_samples,
              dims,
              dist_to_nearest_neighbour,
              step_length,
              lims = pe2.lims,
              collision_func=pe2.test_collisions)

    plan = prm.build_prm(pe2.start, pe2.goal)
    run_time = time.time() - start_time
    print 'plan:', plan
    print 'run_time =', run_time
    pe2.draw_plan(plan,prm,True,True)
    plotter.pause(600)
    return plan, prm

#test_rrt_env()

test_prm_env()