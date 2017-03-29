#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
from __future__ import print_function
import random
import numpy as np
#import matplotlib.pyplot as plotter
from matplotlib import pyplot as plotter
from math import pi, sqrt, sin, cos
from collisions import PolygonEnvironment
import time
from math import sqrt,cos,sin,atan2

_DEBUG = False

_TRAPPED = 'trapped'
_ADVANCED = 'advanced'
_REACHED = 'reached'

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
        connect=True
        self.goal = np.array(goal)
        self.init = np.array(init)
        print ("goal", self.goal)
        print ("start",self.init)
        self.found_path = True
        # Build tree and search
        self.T = RRTSearchTree(init)

        for k in range(1,self.K+1):
            q_rand=self.sample(self.limits,k,self.connect_prob,self.goal)
            q_new_says=self.extend(q_rand,connect)
            if(q_new_says=='breakloop'):
                print ("k:", k)
                break
        (node_list, edge_list)=self.T.get_states_and_edges()
        path=self.T.get_back_path(self.T.nodes[-1])
        return path

    def build_rrt_connect(self, init, goal):
        connect=True
        self.goal = np.array(goal)
        self.init = np.array(init)
        self.found_path = False
        # Build tree and search
        self.T=RRTSearchTree(init) #temporary tree
        self.Ta = RRTSearchTree(init)
        self.Tb = RRTSearchTree(goal)
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
                    print ("k:", k) # here give the plan
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
        print ("Failure")
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

def test_rrt_env(num_samples=500, step_length=2, env='./env0.txt', connect=False):
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
    print ('plan:', plan)
    print ('run_time =', run_time)
    pe.draw_plan(plan,rrt,True,True)
    plotter.pause(600)
    return plan, rrt

test_rrt_env()