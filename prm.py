#!/usr/bin/env python
'''
Package providing helper classes and functions for performing graph search operations for planning.
'''
import numpy as np
import matplotlib.pyplot as plotter
from math import pi
from collisions import PolygonEnvironment
import time
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import KDTree
import rrt
import heapq

_DEBUG = False

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
        return x.state in self.s

    def push(self, x, cost):
        '''
        Adds an element to the priority queue.
        If the state already exists, we update the cost
        '''
        if x.state in self.s:
            return self.replace(x, cost)
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

class SearchNode:
    def __init__(self, s, parent=None, parent_path=None, cost=0.0):
        '''
        s - the state defining the search node
        A - list of actions
        parent - the parent search node
        parent_action - the action taken from parent to get to s
        '''
        self.parent = parent
        self.cost = cost
        self.mapnode = s
        self.parent_path = parent_path
        self.state = tuple(self.mapnode.state)

class Node:
    def __init__(self, state):
        self.state = state
        self.connections = []
        self.connection_paths = []

    def add_connection(self, connect, connection_path):
        #adds a connection
        #adds a path between the connections in the form of a list of subnodes
        if connect not in self.connections:
            self.connections.append(connect)
            self.connection_paths.append(connection_path)

class RoadMap:
    def __init__(self):
        self.nodes = []
        self.edges = []
        self.start = None
        self.goal = None

    def make_kd_tree(self):
        (sts, edg) = self.get_states_and_edges()
        self.KD = KDTree(sts)

    def find_nearest_neighbors(self, s_query, n_neighbors):
        #query tree
        (d, nn_ind) = self.KD.query(s_query, n_neighbors+1)
        #print d
        #print nn_ind
        #get nearest neighbors
        nn = []
        #print "node list length"
        #print len(self.nodes)
        for i in range(len(nn_ind)):
            if i != 0:
                #print nn_ind[i]
                nn.append(self.nodes[nn_ind[i]])
        return (d, nn)

    def add_node(self, node):
        self.nodes.append(node)

    def add_edge(self, node1, node2):
        new_edge = (node1.state, node2.state)
        if new_edge not in self.edges:
            self.edges.append(new_edge)

    def get_states_and_edges(self):
        states = np.array([n.state for n in self.nodes])
        return (states, self.edges)

    def add_start_and_goal(self, init, goal, n_nearest, planner):
        self.start = Node(init)
        self.goal = Node(goal)

        (d_start, adj_start) = self.find_nearest_neighbors(self.start.state, n_nearest)
        for i in range(len(adj_start)):
            dist = np.linalg.norm(self.start.state - adj_start[i].state)
            new_conn = planner(self.start, adj_start[i], dist)
            # if a connection was found
            if new_conn is not None:
                # add the connection to the node
                self.start.add_connection(adj_start[i], new_conn)
                # add the edge to the graph
                self.edges.append((self.start.state, adj_start[i].state))
        (d_goal, adj_goal) = self.find_nearest_neighbors(self.goal.state, n_nearest)
        for i in range(len(adj_goal)):
            dist2 = np.linalg.norm(self.goal.state - adj_goal[i].state)
            new_conn2 = planner(self.goal, adj_goal[i], dist2)
            # if a connection was found
            if new_conn2 is not None:
                # add the connection to the node
                self.goal.add_connection(adj_goal[i], new_conn2)
                #add the goal node as a connection to the nodes
                new_conn3 = [new_conn2[len(new_conn2)-j-1] for j in range(len(new_conn2))]
                adj_goal[i].add_connection(self.goal, new_conn3)
                # print "newconn"
                # print new_conn2
                # print new_conn3
                # add the edge to the graph
                self.edges.append((self.goal.state, adj_goal[i].state))

    def remove_start_and_goal(self):
        num_remove_start = len(self.start.connections)
        num_remove_goal = len(self.goal.connections)
        num_remove = num_remove_start + num_remove_goal
        for i in range(num_remove):
            tempvar = self.edges.pop()
        for nd in self.goal.connections:
            tv1 = nd.connections.pop()
            tv2 = nd.connection_paths.pop()
        self.start = None
        self.goal = None



class PRM:

    def __init__(self, num_samples, num_dimensions=2, n_neighbors = 10, lims = None, collision_func=None, epsilon = 2):
        '''
        Initialize a PRM planning instance
        '''
        self.K = num_samples
        self.n = num_dimensions
        self.nn = n_neighbors
        self.eps = epsilon

        self.in_collision = collision_func
        if collision_func is None:
            self.in_collision = self.fake_in_collision

        # Setup range limits
        self.limits = lims
        if self.limits is None:
            self.limits = []
            for n in xrange(num_dimensions):
                self.limits.append([0,100])
            self.limits = np.array(self.limits)

        self.ranges = self.limits[:,1] - self.limits[:,0]
        self.found_path = False

    def build_roadmap(self, gaussian=False, stdv=2, run_rrt=False, rrt_env='env0.txt'):
        '''
        Build the prm from init to goal
        Returns the road map
        '''

        # Initialize roadmap
        self.M = RoadMap()

        #take samples
        for i in range(self.K):
            # take sample in the area
            if gaussian:
                samp = self.gaussian_sample(stdv)
                #check collision on both
                coll1 = self.in_collision(samp[0])
                coll2 = self.in_collision(samp[1])
                if coll1 != coll2:
                    if coll1:
                        new_node = Node(samp[1])
                        self.M.add_node(new_node)
                    else:
                        new_node = Node(samp[0])
                        self.M.add_node(new_node)
            else:
                samp = self.sample()
                # check collision
                if not self.in_collision(samp):
                    # add to the roadmap
                    new_node = Node(samp)
                    self.M.add_node(new_node)
            # if i == 0:
            #     samp = np.array([-75,-25])
            # if i == 1:
            #     samp = np.array([-75,0])
            # if i == 2:
            #     samp = np.array([-50,0])
            # if i == 3:
            #     samp = np.array([-50,-25])


        print "sampling complete"
        if self.nn > len(self.M.nodes):
            self.nn = len(self.M.nodes)-1
        #connect the nodes
        self.M.make_kd_tree() #make KD tree
        #connect to nearest neighbors for each node
        j = 1
        for n_i in self.M.nodes:
            print j
            #find the nearest neighbors
            (d, nrst) = self.M.find_nearest_neighbors(n_i.state, self.nn)
            #print d
            #print n_i.state
            # for k in nrst:
            #     print k.state
            #connect to the node
            if run_rrt:
                for i in range(len(nrst)):
                    (new_conn, rrtplanner) = rrt.test_rrt_env(env=rrt_env, makeplot=False, start_goal=(n_i.state, nrst[i].state))
                    # if a connection was found
                    if new_conn is not None:
                        # add the connection to the node
                        n_i.add_connection(nrst[i], new_conn)
                        # add the edge to the graph
                        self.M.edges.append((n_i.state, nrst[i].state))
            else:
                for i in range(len(nrst)):
                    dist = np.linalg.norm(n_i.state - nrst[i].state)
                    new_conn = self.local_planner(n_i, nrst[i], dist)
                    #if a connection was found
                    if new_conn is not None:
                        #add the connection to the node
                        n_i.add_connection(nrst[i], new_conn)
                        #add the edge to the graph
                        self.M.edges.append((n_i.state, nrst[i].state))
            j = j+1

    def sample(self):
        '''
        Sample a new configuration and return
        '''
        # Return goal with connect_prob probability
        # Sample randomly in n dimensions within range limits
        x_samp = np.array([np.random.uniform(self.limits[i][0], self.limits[i][1]) for i in range(self.n)])
        return x_samp

    def gaussian_sample(self, stdv):
        '''
        Sample a new configuration and return
        '''
        # Return goal with connect_prob probability
        # Sample a normal distribution in n dimensions within range limits
        x_samp1 = np.array([np.random.uniform(self.limits[i][0], self.limits[i][1]) for i in range(self.n)])
        #take a sample in the gaussian distribution around the first sample
        x_samp2 = np.array([np.random.normal(x_samp1[i], stdv) for i in range(self.n)])
        x_samp = (x_samp1, x_samp2)
        return x_samp

    def local_planner(self, n1, n2, dist):
        '''
        straight-line local planner for connecting nodes
        '''
        intmd_states = []
        #find unit vector in direction between nodes
        #note: the KD tree nearest neighbors will return the node itself as a nearest neighbor, so don't include it.
        if dist != 0:
            vhat = (n2.state-n1.state)/dist
            #multiply unit vector by travel distance
            v_travel = vhat*self.eps
            #add to the nearest node to get the new node
            q_new_state = n1.state
            k = 1
            while np.linalg.norm(q_new_state - n2.state) > self.eps:
                #print k
                if k > 200:
                    print vhat
                    print v_travel
                    print dist
                    print np.linalg.norm(q_new_state - n2.state)
                    print n1.state
                    print n2.state
                    print "error"
                    return None
                #check collision
                if not self.in_collision(q_new_state):
                    #if not in collision, add state to the list of intermediate states
                    intmd_states.append(q_new_state)
                else:
                    #otherwise return "none"
                    return None
                # advance state by epsilon
                q_new_state = q_new_state + v_travel
                k = k+1
            intmd_states.append(n2.state)
            return intmd_states
        else:
            return None

    def A_star(self, init, goal, h):
        '''
        A* planner for planning a path along the roadmap
        inputs: initial and goal nodes
        output: path
        '''
        front = PriorityQ()  # frontier, as a priority queue
        visited_cost = []  # visited list with costs included
        if init.connections is None or goal.connections is None:
            return None
        init_node = SearchNode(init)  # initial node
        init_heuristic = h(init.state, goal.state)  # heuristic for initial state
        front.push(init_node, init_heuristic)  # add initial node to frontier
        while len(front) > 0:
            # get the lowest cost node
            current_node = front.pop()
            # check whether it's already been visited at a lower cost
            if not self.in_visited_cost(current_node, visited_cost):
                # add the state to the visited list with its cost
                visited_cost.append((current_node.state, current_node.cost))
                # check whether the state is the goal
                if self.is_goal(current_node.state, goal.state):
                    # get the path
                    (path, parent_path) = self.backpath(current_node)
                    return (path, parent_path)
                else:
                    # loop through the actions
                    for j, nd in enumerate(current_node.mapnode.connections):
                        # get new cost
                        new_cost = current_node.cost + np.linalg.norm(nd.state - current_node.state)
                        # make new node
                        # print current_node.mapnode.connection_paths[j]
                        n_prime = SearchNode(nd, parent=current_node, parent_path=current_node.mapnode.connection_paths[j], cost=new_cost)
                        # get heuristic
                        heuristic_cost = h(n_prime.state, goal.state)
                        # add heuristic and cost
                        total_cost = heuristic_cost + new_cost
                        # add the node to the frontier only if it's not in the frontier at a lower cost
                        if n_prime in front:
                            if front.get_cost(n_prime) < total_cost:
                                pass
                                print "not adding node"
                            else:
                                print "adding visited node at lower cost"
                                front.push(n_prime, total_cost)
                        else:
                            print "adding unvisited node"
                            front.push(n_prime, total_cost)
        return None

    def euclidean_heuristic(self, s, goal):
        '''
        Heuristic for Euclidean distance

        s - tuple describing the state as (row, col) position on the grid.

        returns - floating point estimate of the cost to the goal from state s
        '''
        np.array(s)
        np.array(goal)
        h = np.linalg.norm(s - goal)

        return h

    def in_visited_cost(self, current_node, visited):
        # checks whether a node is in the visited list at a lower cost, returns true if so and false if not
        for i in range(len(visited)):
            if current_node.state == visited[i][0] and current_node.cost > visited[i][1]:
                return True

        return False

    def is_goal(self, s, goal):
        isgoal = True
        for i in range(len(s)):
            if s[i] != goal[i]:
                isgoal = False
        return isgoal

    def backpath(self, node):
        '''
        Function to determine the path that lead to the specified search node

        node - the SearchNode that is the end of the path

        returns - a tuple containing (path, action_path) which are lists respectively of the states
        visited from init to goal (inclusive) and the actions taken to make those transitions.
        '''
        path = []
        parent_path = []
        # Fill me in!
        current_node = node  # initialize node to look at
        # keep going backwards until the beginning of the path is reached
        while current_node.parent != None:
            print current_node.state
            print current_node.parent_path
            path.insert(0, current_node.state)  # add the current state to the path
            for i in range(len(current_node.parent_path)):
                parent_path.insert(0, current_node.parent_path[len(current_node.parent_path)-i-1])  # add the action to get to this state to the action path
            current_node = current_node.parent  # set the new search node as the previous node
        # add the start state to the path
        path.insert(0, current_node.state)
        return (path, parent_path)

    def fake_in_collision(self, q):
        '''
        We never collide with this function!
        '''
        return False

def test_prm_env(num_samples=50, step_length=2, n_neighbors=5, env='./env0.txt', plotname='plot', gaussian=False, stdv=None, env2=None, env3=None, run_rrt=False):
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

    dims = len(pe.start)
    start_time = time.time()

    prm = PRM(num_samples,
              dims,
              n_neighbors,
              lims = pe.lims,
              epsilon=step_length,
              collision_func=pe.test_collisions)

    if stdv is not None:
        gstd = stdv
    else:
        gstd = 3*step_length
    if run_rrt:
        prm.build_roadmap(gaussian=gaussian, stdv=gstd, run_rrt=True, rrt_env=env)
    else:
        prm.build_roadmap(gaussian=gaussian, stdv=gstd)
    plan = None
    mapplotname = plotname + '_map.jpg'
    pe.draw_plan_prm(plan, prm, fname=mapplotname)
    print "drawn map"
    prm.M.add_start_and_goal(pe.start, pe.goal, n_neighbors, prm.local_planner)
    print "starting planning"
    (plan, subplan) = prm.A_star(prm.M.start, prm.M.goal, prm.euclidean_heuristic)
    print "done planning"
    plotname1 = plotname + '.jpg'
    pe.draw_plan_prm(subplan, prm, fname=plotname1, draw_goal=True)

    run_time = time.time() - start_time
    print 'plan:', plan
    print 'subplan:', subplan
    print 'run_time =', run_time
    #pe.draw_plan_prm(plan, rrt, fname=plotname)

    if env2 is not None:
        pe2 = PolygonEnvironment()
        pe2.read_env(env2)

        prm.M.remove_start_and_goal()
        prm.M.add_start_and_goal(pe2.start, pe2.goal, n_neighbors, prm.local_planner)
        (plan2, subplan2) = prm.A_star(prm.M.start, prm.M.goal, prm.euclidean_heuristic)
        plotname2 = plotname + '2.jpg'
        pe2.draw_plan_prm(subplan2, prm, fname=plotname2, draw_goal=True)

    if env3 is not None:
        pe3 = PolygonEnvironment()
        pe3.read_env(env3)

        prm.M.remove_start_and_goal()
        prm.M.add_start_and_goal(pe3.start, pe3.goal, n_neighbors, prm.local_planner)
        (plan3, subplan3) = prm.A_star(prm.M.start, prm.M.goal, prm.euclidean_heuristic)
        plotname3 = plotname + '3.jpg'
        pe3.draw_plan_prm(subplan3, prm, fname=plotname3, draw_goal=True)



    return plan, prm
