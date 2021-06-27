import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from numpy import random
from numpy.core.numeric import Inf
from numpy.linalg.linalg import det

class q_class:
    def __init__(self, p, sigma, id, t, u):
        # position of agents
        self.p = p
        # uncertainity of measurements
        self.sigma = sigma
        # unique id of the node in the graph
        self.id = id
        # timestep of the node of the graph
        self.t = t
        # cost of the node
        self.cost = 0
        # action performed to reach that node from its parent
        self.u = u

    def update_cost(self, previous_cost):
        self.cost = previous_cost + np.linalg.det(self.sigma)

class Environment:
    def __init__(self, width, height, t, visualize=True):
        
        self.width = width
        self.height = height
        self.t = t

        # ROW: agent/target, COL: pos x pos y
        self.qi = np.zeros((0,2))
        #self.sigma_agents = np.zeros((0,2)) No motion noise
        self.n_agents = 0
        self.u_agents = np.zeros((0,2))
        self.hist_qi = []

        self.xi = np.zeros((0,2))  
        self.sigma_targets = np.zeros((0,2))
        self.n_targets = 0
        self.u_targets = np.zeros((0,2))
        self.hist_xi = []

        # Unique incremental id for the nodes in the graph
        self.id_nodes_graph = 0
        # Uncertainity of measurements
        self.sigma_estimated = []
        # Estimated position of the 
        self.x_estimated = []
        # Secuence of actions to perform
        self.u_path = []

        # PARAMETERS TO TUNE
        # Number of trials of the algorithm
        self.N_sampling = 5
        # Minimum node cost admissible as solution
        self.delta = 0.18

        if visualize:
            # Configure plot
            print("Goes in")
            plt.plot(0,0)
            self.ani = FuncAnimation(plt.gcf(), self.plot_env, interval=self.t*1000)
            
    def add_agent(self, x, y, sigma_x, sigma_y):
        self.qi = np.vstack((self.qi, np.array([x,y])))
        self.u_agents = np.vstack((self.u_agents, np.array([0,0])))
        # Add commands of action (0,0) for the new agent until the current sequence is finished
        for i in range(len(self.u_path)):
            self.u_path[i] = np.vstack((self.u_path[i], np.array([0,0])))
        self.n_agents += 1
    
    def add_target(self, x, y, sigma_x, sigma_y):
        self.xi = np.vstack((self.xi, np.array([x,y])))
        self.u_targets = np.vstack((self.u_targets, (np.random.random((1,2))-0.5)*5))
        self.sigma_targets = np.vstack((self.sigma_targets, np.array([sigma_x, sigma_y])))
        self.n_targets += 1
        # Add position estimated to the matrix
        self.x_estimated.append(x)
        self.x_estimated.append(y)
        # Add sigma estimated to the matrix
        sigma_aux = np.identity(len(self.sigma_estimated)+2)
        if np.size(self.sigma_estimated) != 0:
            sigma_aux[:np.shape(self.sigma_estimated)[0],:np.shape(self.sigma_estimated)[1]] = self.sigma_estimated
        sigma_aux[-2, -2] = sigma_x
        sigma_aux[-1, -1] = sigma_y
        self.sigma_estimated = sigma_aux

    def apply_kalman_filter(self, pos_q, sigma_in):
        # Use a Kalman filter to obtain measurement and covariance of the 
        # measurements of the targets. We assume that the robot possition is known
        # and that every robot can measure all the targets, but the covariance of
        # the measurement is proportional to the distance to the target.
        sigma = sigma_in # Use the sigma received as a parameter instead of the one in the class,
                         # to be able to get the sigma estimated after applying the kalman filter
                         # without modifying the class (in update we need to modify it, but in the active
                         # information acquisition no)
        x = self.x_estimated
        # Apply one Kalman filter for each of the agents (each agent performs a measurement)
        for agent in range(self.n_agents):
            measurements_sigma = np.identity(self.n_targets*2)
            measurements_x = np.zeros((self.n_targets*2, 1))
            for target in range(self.n_targets):
                difx = pos_q[agent, 0] - self.xi[target, 0]
                dify = pos_q[agent, 1] - self.xi[target, 1]
                # Construct uncertainity of the current measurements, depending on the distance
                measurements_sigma[2*target, 2*target] = 0.1 * np.sqrt(difx**2 + dify**2)
                measurements_sigma[2*target + 1, 2*target+1] = 0.001 * (difx**2 + dify**2)
                measurements_x[2*target] = np.random.randn()*measurements_sigma[2*target, 2*target]
                measurements_x[2*target+1] = np.random.randn()*measurements_sigma[2*target+1, 2*target+1]
            # Apply the Kalman filter
            y = measurements_x - x
            S = sigma + measurements_sigma
            K = sigma@np.linalg.inv(S)
            x = x + K*y
            sigma = (np.identity(len(K))-K) * sigma
        return x, sigma

    def update(self):
        '''
            Update the environment dynamics (agents and state)
        '''
        # Update agents according to Eq(1)
        self.hist_qi.append(self.qi)
        if self.n_agents > 0:
            # If new execution or previous path has finished
            if (len(self.u_path) == 0):
                self.u_path = self.sampling_based_active_information_acquisition()
            self.set_agents_command(self.u_path[0])
            self.u_path.pop(0)
        self.qi += self.u_agents #* self.t
        # Update state (targets) according to Eq(2)
        self.hist_xi.append(self.xi)
        noise_targets = np.random.randn(self.xi.shape[0], self.xi.shape[1]) * self.sigma_targets
        self.xi += self.u_targets * self.t + noise_targets
        # Kalman filter to update measurements
        x, sigma = self.apply_kalman_filter(self.qi, self.sigma_estimated)
        self.x_estimated = x
        self.sigma_estimated = sigma

    def set_agents_command(self, u_agents):
        self.u_agents = u_agents

    def set_targets_command(self, u_targets):
        self.u_targets = u_targets

    def plot_env(self, _):
        plt.cla()
        plt.xlim(-self.width/2, self.width/2)
        plt.ylim(-self.height/2, self.height/2)
        if(self.qi.shape[0] > 0):
            plt.scatter(self.qi[:,0], self.qi[:,1], 4, 'b', 'o')
        if(self.xi.shape[0] > 0):
            plt.scatter(self.xi[:,0], self.xi[:,1], 4, 'r', 'x')
    
    def free_space(self, p):
        # In our case, we assume that there is no inaccessible space
        return True

    def sample_fv(self, V, max_t):
        # Sample a configuration of the graph (one or several nodes with the same configuration,
        # having that the configuration is the position of the agents. There may be several nodes, as 
        # two nodes with the same configuration may have different cost)
        sampling_vector = []
        for i in range(len(V)):
            found = False
            for q in V[i]:
                if q.t == max_t:
                    found = True
            sampling_vector.append(i)
            # Give more probability to Kmax
            if found:               
                sampling_vector.append(i)
        return random.choice(sampling_vector)
    
    def sample_fu(self):
        # In the approach of the paper, where robots have a limited sensor range,
        # the approach taken is to bring each robot closer to each target until
        # they may sense the target. In our case, this "cheat" is not needed, as we
        # use an infinite sensor range, and the np.size(V[i])preferred final u will be always selected
        # using the covariance of the measurements. Thus, the sampling of u is completely
        # random.
        u_possibilities = [0.2, -0.2]
        return random.choice(u_possibilities, (np.shape(self.u_agents)))

    def same_configuration(self, q, V):
        # Check if there is a group of nodes with the same configuration as q.
        # Returns if whether it has been found and the k corresponding to the group.
        k = -1
        found = False
        for i in range(len(V)):
            equal = True
            for j in range(len(q.p)):
                if any(q.p[j] != V[i][0].p[j]):
                    equal = False
            if equal:
                found = True
                k = i
        return found, k

    def find_recover_path(self, X, V, epsilon):
        # Get the node with the smallest cost and returns the sequence of actions to get to that node
        lowest_cost = float("inf")
        lowest_id = -1
        for q in X:
            if q.cost < lowest_cost:
                lowest_cost = q.cost
                lowest_id = q.id
        finished = False
        path_q = [lowest_id]
        while not finished:
            for [parent, child] in epsilon:
                if child == lowest_id and V[parent].t == 0:
                    finished = True
                    break
                elif child == lowest_id:
                    lowest_id = parent
                    path_q.append(parent)
                    break
        path_q.reverse()
        u = []
        for q_id in path_q:
            u.append(V[q_id].u)
        return u


    def sampling_based_active_information_acquisition(self):
        # p: state of the robots (position in our case)
        # u: control of the robots
        # x: hidden state (position of mobile objects). Noise with covariance Q(t)
        # y: measurement, covariance R(t)
        # mu: mean after fusing measurements
        # sigma: mu after fusing measurements
        # ro: Kalman filter Ricatti map
        # V: nodes of the tree. Each node is q(t) = [p(t), sigma(t)]
        # epsilon: edges of the tree
        # Xg: set of states that collects all states
        q_0 = q_class(self.qi, self.sigma_estimated, self.id_nodes_graph, 0, np.zeros(np.shape(self.u_agents))) # Node corresponding to the root
        self.id_nodes_graph = self.id_nodes_graph+1
        q_0.update_cost(0)
        V_total = {q_0.id: q_0} # Dictionary that stores the nodes with their id as the key
        Vk = [[q_0]] # Nodes of the graph organised by configurations
        epsilon = [] # edges
        K = 0
        Xg = []  # Nodes that may be the solution
        N = self.N_sampling # Number of trials
        max_t = 0
        for n in range(N):
            Vkrand = self.sample_fv(Vk, max_t) # Sample a k (corresponds to a position of 1 or more nodes with the same configuration)
            u_new = self.sample_fu() # Sample an action
            p_new = Vk[Vkrand][0].p + u_new # New position with that action
            if self.free_space(p_new):
                for q_rand in Vk[Vkrand]: # Apply it for all the nodes with that configuration
                    x, sigma_new = self.apply_kalman_filter(q_rand.p, q_rand.sigma) # get uncertainity in the new configuration
                    q_new = q_class(p_new, sigma_new, self.id_nodes_graph, q_rand.t + 1, u_new) # create new node of the graph
                    self.id_nodes_graph = self.id_nodes_graph+1 # Increment the unique id
                    q_new.update_cost(q_rand.cost)  # compute the cost of the new node
                    max_t = max(max_t, q_new.t)     # keep the length of the maximum sequence
                    V_total[q_new.id] = q_new       # add node to the dictionary
                    epsilon.append([q_rand.id, q_new.id])   # add edge of the tree
                    exists, k = self.same_configuration(q_new, Vk)  # Check if there are nodes with the same configuration
                    # Add the node where it corresponds regarding the configuration
                    if exists:
                        Vk[k].append(q_new)
                    else:
                        K = K+1
                        Vk.append([q_new])
                    # Add node to candidates
                    if np.linalg.det(q_new.sigma) <= self.delta:
                        Xg.append(q_new)
        # Get the sequence of actions to follow
        path = self.find_recover_path(Xg, V_total, epsilon)
        return path


