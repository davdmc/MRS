import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from numpy import random
from numpy.core.numeric import Inf
from numpy.linalg.linalg import det

class AIANode:
    def __init__(self, p, cov):
        self.id = None
        self.timestamp = None
        self.p = p
        self.cov = cov
        self.cost = None
        self.children = []
        self.parents = []
        self.reaching_motion = None

class AIATree:
    def __init__(self, root):
        assert root.id == 0 and root.timestamp == 0 and root.reaching_motion != None and root.cost != None, "Root not properly initialized"
        root.cost = Inf #!!! Check what is sigma 0
        self.nodes = [root] # Hold a reference to all the nodes
        self.edges = [] # Edges are stored as: from-to tuple of nodes. However, nodes are connected by the Node class.
        self.id_count = 1 # This id is incremented every time a new child is added
        self.v_k = {root.p:[root]} # v_k are the sets of nodes with the same robot-configuration. Main way of sampling nodes.
        self.l_max = 1 # Max number of hops
        self.l_max_set = [root] # Set of all the nodes that are at L_max 

    def append_child(self, parent, child, reach_cost):
        '''
            Inserts a new child and maintains the tree structure while doing it
        '''
        assert child.reaching_motion != None, "Node not properly initialized"
        ## Init child node in the tree
        # Assign an unique id
        child.id = self.id_count
        self.id_count += 1
        # Assign one more timestamp == hops in the tree == branch length
        child.timestamp = parent.timstamp + 1
        # Compute cost to access the node
        child.cost = parent.cost + reach_cost

        # Add node and edge
        self.nodes.append(child)
        self.edges.append((parent, child))

        ## Link nodes
        parent.children.append(child)
        child.parents.append(parent)

        ## Maintain the l_max and l_max_set in order to apply the biased sample of f_v
        if child.timestamp > self.l_max:
            self.l_max = child.timestamp
            self.l_max_set = [child]
        elif child.timestamp == self.l_max:
            self.l_max_set.append(child)

        # Introduce it into v_k or create a new v_k
        if child.p in self.v_k.items()[0]:
            self.v_k[self.p].append(child)
        else:
            self.v_k[self.p] = [child]


    def sample_v(self, p_v = 0.7):
        '''
            The sampling strategy is based on sec V.A: gives more priority to leaves
        '''
        ## Construct k_max and V\k_max checking if the V in K_n are in l_max_set
        k_max = []
        k_other = []
        # Iterate over the robot-configurations in v_k
        for key, values in self.v_k.items():
            is_k_max = False
            for value in values:
                # If any is in the set of L_max, the configuration is in k_max
                if value in self.l_max_set:
                    is_k_max = True
                    k_max.append(key)
                    break
            if not is_k_max:
                k_other.append(key)
        
        # Construct a weight vector for sampling probabilities
        k_all = k_max + k_other
        weight_max = p_v * 1/len(k_max)
        weight_other = p_v * 1/len(k_other)
        weights = []
        for i in range(len(k_all)):
            if i < len(k_max):
                weights.append(weight_max)
            else:
                weights.append(weight_other)
        return random.choice(k_all, p=weights)

    def get_min_path(self):
        '''
            Searches the minimum cost node and gets the path to get to it
        '''
        ## Find the minimum node cost
        min_cost_node = self.nodes[0]
        for node in self.nodes:
            if node.cost < min_cost_node.cost:
                min_cost_node = node

        ## Get the path from that node to the root and reverse it
        path = [min_cost_node.reaching_motion]
        current_node = min_cost_node
        while current_node.parent != None:
            current_node = current_node.parent
            path.append(current_node.reaching_motion)
        
        return path.reverse()

def kalman_filter(x_old, z, P_old, Q, R):
    '''
        Apply the kalman filter with an unknown state transitions (no velocities known from moving targets)
        and a noisy observation model that returns the position from every target: z = I x + w -> H = I
    '''
    H = np.eye(len(x_old))

    # Predict
    x_est = x_old
    P_est = P_old + Q

    # Update
    y = z - H @ x_est
    K = P_est @ np.linalg.inv(R + H @ P_est @ H.T)
    x_new = x_est + K @ y
    tmp = K @ H
    P_new = (np.eye(len(tmp) - tmp)) @ P_est

    return x_new, P_new

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

        # Real state values
        self.xi = np.zeros((0,2))
        self.sigma_targets = np.zeros((0,2))
        self.n_targets = 0
        self.u_targets = np.zeros((0,2))
        # Estimated state values
        self.x_est = np.zeros((0,2))
        self.P_est = np.zeros((0,0))

        self.hist_xi = []

        
        # Estimated position of the 
        
        # Secuence of actions to perform
        self.u_path = []

        # PARAMETERS TO TUNE
        # Number of trials of the algorithm
        self.N_sampling = 5
        # Minimum node cost admissible as solution
        self.delta = 0.18

        if visualize:
            # Configure plot
            plt.plot(0,0)
            self.ani = FuncAnimation(plt.gcf(), self.plot_env, interval=self.t*1000)
            
    def add_agent(self, x, y, sigma_x, sigma_y):
        # Add 
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
        self.sigma_targets = np.vstack((self.sigma_targets, np.array([sigma_x, sigma_y])))
        # Add sigma estimated to the matrix
        P_sigmas = diagonal(P)

    def update(self):
        '''
            Update the environment dynamics (agents and state)
        '''
        # Update agents according to Eq(1)
        self.hist_qi.append(self.qi)
        self.qi += self.u_agents # * self.t Not needed since controls are motion primitives

        # Update state (targets) according to Eq(2)
        self.hist_xi.append(self.xi)
        noise_targets = np.random.randn(self.xi.shape[0], self.xi.shape[1]) * self.sigma_targets
        self.xi += self.u_targets * self.t + noise_targets
    

    # def apply_kalman_filter(self, pos_q, sigma_in):
    #     # Use a Kalman filter to obtain measurement and covariance of the 
    #     # measurements of the targets. We assume that the robot possition is known
    #     # and that every robot can measure all the targets, but the covariance of
    #     # the measurement is proportional to the distance to the target.
    #     sigma = sigma_in # Use the sigma received as a parameter instead of the one in the class,
    #                      # to be able to get the sigma estimated after applying the kalman filter
    #                      # without modifying the class (in update we need to modify it, but in the active
    #                      # information acquisition no)
    #     x = self.x_estimated
    #     # Apply one Kalman filter for each of the agents (each agent performs a measurement)
    #     for agent in range(self.n_agents):
    #         measurements_sigma = np.identity(self.n_targets*2)
    #         measurements_x = np.zeros((self.n_targets*2, 1))
    #         for target in range(self.n_targets):
    #             difx = pos_q[agent, 0] - self.xi[target, 0]
    #             dify = pos_q[agent, 1] - self.xi[target, 1]
    #             # Construct uncertainity of the current measurements, depending on the distance
    #             measurements_sigma[2*target, 2*target] = 0.1 * np.sqrt(difx**2 + dify**2)
    #             measurements_sigma[2*target + 1, 2*target+1] = 0.001 * (difx**2 + dify**2)
    #             measurements_x[2*target] = np.random.randn()*measurements_sigma[2*target, 2*target]
    #             measurements_x[2*target+1] = np.random.randn()*measurements_sigma[2*target+1, 2*target+1]
    #         # Apply the Kalman filter
    #         y = measurements_x - x
    #         S = sigma + measurements_sigma
    #         K = sigma@S
    #         x = x + K*y
    #         sigma = (np.identity(len(K))-K) * sigma
    #     return x, sigma

    # def update_agents_command(self):
    #     if self.n_agents > 0:
    #         # If new execution or previous path has finished
    #         if (len(self.u_path) == 0):
    #             self.u_path = self.sampling_based_active_information_acquisition()
    #         self.set_agents_command(self.u_path[0])
    #         self.u_path.pop(0)

    def set_agents_command(self, u_agents):
        self.u_agents = u_agents

    def set_targets_command(self, u_targets):
        self.u_targets = u_targets
    
    def free_space(self, p):
        # In our case, we assume that there is no inaccessible space
        return True
    
    def sample_fu(self):
        # In the approach of the paper, where robots have a limited sensor range,
        # the approach taken is to bring each robot closer to each target until
        # they may sense the target. In our case, this "cheat" is not needed, as we
        # use an infinite sensor range, and the np.size(V[i])preferred final u will be always selected
        # using the covariance of the measurements. Thus, the sampling of u is completely
        # random.
        # TODO: implement sensor range and on-the-fly target assignment
        u_possibilities = [0.2, -0.2]
        return random.choice(u_possibilities, (np.shape(self.u_agents)))

    def plot_env(self, _):
        plt.cla()
        plt.xlim(-self.width/2, self.width/2)
        plt.ylim(-self.height/2, self.height/2)
        if(self.qi.shape[0] > 0):
            plt.scatter(self.qi[:,0], self.qi[:,1], 4, 'b', 'o')
        if(self.xi.shape[0] > 0):
            plt.scatter(self.xi[:,0], self.xi[:,1], 4, 'r', 'x')

def sampling_based_active_information_acquisition(max_n, environment):
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
    
    Xg = []  # Nodes that may be the solution
    root = AIANode(environment.p)
    tree = AIATree()
    for n in range(max_n):
        vk_rand = tree.sample_fv() # Sample a k (corresponds to a position of 1 or more nodes with the same configuration)
        u_new = environment.sample_fu() # Sample an action
        p_new = vk_rand + u_new # New position with that action
        if environment.free_space(p_new):
            for q_rand in tree.v_k[vk_rand]: # Apply it for all the nodes with that configuration
                x, sigma_new = kalman_filter(q_rand.p, q_rand.sigma) # get uncertainity in the new configuration
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

   