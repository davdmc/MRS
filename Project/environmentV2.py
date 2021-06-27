import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from numpy import random
from numpy.core.numeric import Inf
from numpy.linalg.linalg import det

def sigma_measure(l):
    return 0.0001 + l * 0.075
        
class AIANode:
    def __init__(self, p, x_est, cov, reaching_motion):
        self.id = None
        self.timestamp = None
        self.p = p
        self.x_est = x_est
        self.cov = cov
        self.cost = None
        self.children = []
        self.parents = []
        self.reaching_motion = reaching_motion

class AIATree:
    def __init__(self, root):
        assert root.id == 0 and root.timestamp == 0 and root.reaching_motion.all() != None and root.cost.all() != None, "Root not properly initialized"
        #root.cost = 0 #!!! Check what is sigma 0
        self.nodes = [root] # Hold a reference to all the nodes
        self.edges = [] # Edges are stored as: from-to tuple of nodes. However, nodes are connected by the Node class.
        self.id_count = 1 # This id is incremented every time a new child is added
        self.v_k = {(root.p).tostring():[root]} # v_k are the sets of nodes with the same robot-configuration. Main way of sampling nodes.
        self.t_max = 0 # Max number of hops
        self.t_max_set = [root] # Set of all the nodes that are at L_max 

    def append_child(self, parent, child, reach_cost):
        '''
            Inserts a new child and maintains the tree structure while doing it
        '''
        assert child.reaching_motion.all() != None, "Node not properly initialized"
        ## Init child node in the tree
        # Assign an unique id
        child.id = self.id_count
        self.id_count += 1
        # Assign one more timestamp == hops in the tree == branch length
        child.timestamp = parent.timestamp + 1
        # Compute cost to access the node
        child.cost = parent.cost + reach_cost

        # Add node and edge
        self.nodes.append(child)
        self.edges.append((parent, child))

        ## Link nodes
        parent.children.append(child)
        child.parents.append(parent)

        ## Maintain the t_max and t_max_set in order to apply the biased sample of f_v
        if child.timestamp > self.t_max:
            self.t_max = child.timestamp
            self.t_max_set = [child]
        elif child.timestamp == self.t_max:
            self.t_max_set.append(child)

        # Introduce it into v_k or create a new v_k
        if child.p.tostring() in self.v_k.keys():
            self.v_k[(child.p).tostring()].append(child)
        else:
            self.v_k[(child.p).tostring()] = [child]


    def sample_fv(self, p_v = 0.7):
        '''
            The sampling strategy is based on sec V.A: gives more priority to leaves
        '''
        ## Construct k_max and V\k_max checking if the V in K_n are in t_max_set
        k_max = []
        k_other = []
        # Iterate over the robot-configurations in v_k

        for key, values in self.v_k.items():
            is_k_max = False
            for value in values:
                # If any is in the set of L_max, the configuration is in k_max
                if value in self.t_max_set:
                    is_k_max = True
                    k_max.append(np.fromstring(key, dtype=float))
                    break
            if not is_k_max:
                k_other.append(np.fromstring(key, dtype=float))
        
        # Construct a weight vector for sampling probabilities
        k_all = k_max + k_other
        weight_max = p_v * 1/len(k_max)
        if len(k_other) != 0:
            weight_other = (1 - p_v) * 1/len(k_other)
        else:
            weight_max += (1 - p_v)
            weight_other = 0
        
        weights = []
        for i in range(len(k_all)):
            if i < len(k_max):
                weights.append(weight_max)
            else:
                weights.append(weight_other)

        idx = random.choice(len(k_all), p=weights)
        return k_all[idx]

    def get_min_path(self, delta):
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
        while len(current_node.parents) != 0:
            current_node = current_node.parent
            path.append(current_node.reaching_motion)
        path.reverse()
        return path

def kalman_predict(x_old, P_old, Q):
    '''
        Apply the kalman filter predict with an unknown state transitions (no velocities known from moving targets)
    '''
    x_est = x_old
    P_est = P_old + Q
    return x_est, P_est

def kalman_update(x_est, z, P_est, R):
    '''
        Apply the kalman filter update with noisy observation model that returns the position from every target: 
        z = I x + w -> H = I
    '''
    H = np.eye(len(x_est))

    # Update
    y = z - x_est

    K = P_est @ np.linalg.inv(R + H @ P_est @ H.T)
    x_new = x_est + K @ y
    tmp = K @ H
    P_new = (np.eye(tmp.shape[0]) - tmp) @ P_est
    # print("KALMAN")
    # print(y)
    # print(P_old[0,0] > P_new[0,0])
    return x_new, P_new

class Environment:
    def __init__(self, width, height, t, visualize=True):
        
        self.width = width
        self.height = height
        self.t = t

        # ROW: agent/target, COL: pos x pos y
        self.qi = np.zeros((0,2))
        self.hist_qi = []
        #self.sigma_agents = np.zeros((0,2)) No motion noise
        self.n_agents = 0
        self.u_agents = np.zeros((0,2))

        # Real state values
        self.xi = np.zeros((0,2))
        self.hist_xi = []
        self.sigma_targets = np.zeros((0,2))
        self.n_targets = 0
        self.u_targets = np.zeros((0,2))
        self.Q = []
        # Estimated state values. These are represented differently: serialized
        self.x_est = []
        self.P_est = []

        # Secuence of actions to perform
        self.u_path = []

        # PARAMETERS TO TUNE
        # Number of trials of the algorithm
        self.max_n = 5
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
        self.x_est.append(x)
        self.x_est.append(y)
        # Add cov to the Q matrix
        Q_aux = np.identity(len(self.Q)+2)
        if np.size(self.Q) != 0:
            Q_aux[:np.shape(self.Q)[0],:np.shape(self.Q)[1]] = self.Q
        Q_aux[-2, -2] = sigma_x ** 2
        Q_aux[-1, -1] = sigma_y ** 2
        self.Q = Q_aux
        # Add cov estimated to the matrix
        P_est_aux = np.identity(len(self.P_est)+2)
        if np.size(self.P_est) != 0:
            P_est_aux[:np.shape(self.P_est)[0],:np.shape(self.P_est)[1]] = self.P_est
        P_est_aux[-2, -2] = sigma_x ** 2
        P_est_aux[-1, -1] = sigma_y ** 2
        self.P_est = P_est_aux

    def update(self):
        '''
            Update the environment dynamics (agents and state)
        '''
        self.update_agents_command()

        # Update agents according to Eq(1)
        self.hist_qi.append(self.qi)
        self.qi += self.u_agents # * self.t Not needed since controls are motion primitives

        # Update state (targets) according to Eq(2)
        self.hist_xi.append(self.xi)
        noise_targets = np.random.randn(self.xi.shape[0], self.xi.shape[1]) * self.sigma_targets
        self.xi += self.u_targets * self.t + noise_targets

        z, R = self.get_measurements(self.qi)
        x_est, P_est = kalman_predict(self.x_est, self.P_est, self.Q)
        self.x_est, self.P_est  = kalman_update(x_est, z[0,:], P_est, np.diag(R[0,:])) # get uncertainity in the new configuration
        for robot in range(1,self.n_agents):
            self.x_est, self.P_est = kalman_update(x_est, z[robot,:], P_est , np.diag(R[robot,:])) # get uncertainity in the new configuration
                  

    def update_agents_command(self):
        '''
            Update the agents command before each environemnt update
        '''
        if self.n_agents > 0:
            # If new execution or previous path has finished
            if (len(self.u_path) == 0):
                self.u_path, self.tree = sampling_based_active_information_acquisition(self.max_n, self, self.delta)
            self.set_agents_command(self.u_path[0])
            self.u_path.pop(0)

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
        u_possibilities = [2, -2]
        return random.choice(u_possibilities, (np.shape(self.u_agents)))

    def get_measurements(self, pi):
        '''
            Get the noisy position of the targets by each agent and stores it as follows:
            Row: Different agents
            Columns: Alternate x/y and targets for the same agent
            Also gets the covariance marix depending on the distance and stores it as follows:
            Row: DIfferent agents
            Columns: diagonal of the covariance matrix
        '''
        z = np.zeros((self.n_agents, 2*self.n_targets))
        R = np.zeros((self.n_agents, 2*self.n_targets))
        for j in range(self.n_agents):
            for i in range(self.n_targets):
                dist_x = pi[j,0] - self.xi[i,0]
                dist_y = pi[j,1] - self.xi[i,1]
                dist_l = np.sqrt(dist_x**2 + dist_y**2)
                # Construct uncertainity of the current measurements, depending on the distance
                sigma_x = sigma_measure(dist_l)
                sigma_y = sigma_measure(dist_l)
                z[i, 2*j] = self.xi[i,0] + np.random.random() * sigma_x
                z[i, 2*j+1] = self.xi[i,1] + np.random.random() * sigma_y
                R[i, 2*j] = sigma_x**2
                R[i, 2*j+1] = sigma_y**2

        return z, R
                

    def plot_env(self, _):
        plt.cla()
        plt.xlim(-self.width/2, self.width/2)
        plt.ylim(-self.height/2, self.height/2)
        if(self.n_agents > 0):
            plt.scatter(self.qi[:,0], self.qi[:,1], 4, 'b', 'o')
        if(self.n_targets > 0):
            plt.scatter(self.xi[:,0], self.xi[:,1], 4, 'r', 'x')

        for nodes in self.tree.nodes:
            plt.text(nodes.p[:,0], nodes.p[:,1], "{0:.3g}".format(nodes.cost))

def sampling_based_active_information_acquisition(max_n, environment, delta):
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
    
    x_est = environment.x_est
    P_est = environment.P_est

    root = AIANode(environment.qi, x_est, P_est, np.zeros(environment.qi.shape))
    root.id = 0
    root.timestamp = 0
    root.cov = P_est
    root.cost = np.linalg.det(P_est)

    tree = AIATree(root)

    for n in range(max_n):
        vk_rand = tree.sample_fv() # Sample a k (corresponds to a position of 1 or more nodes with the same configuration)
        u_new = environment.sample_fu() # Sample an action
        p_new = vk_rand + u_new # New position with that action
        if environment.free_space(p_new):
            for q_rand in tree.v_k[(vk_rand).tostring()]: # Apply it for all the nodes with that configuration
                z, R = environment.get_measurements(q_rand.p)
                x_est, P_est = kalman_predict(q_rand.x_est, q_rand.cov, environment.Q)
                x_new, P_new = kalman_update(x_est, z[0,:], P_est, np.diag(R[0,:])) # get uncertainity in the new configuration
                for robot in range(1,environment.n_agents):
                    x_new, P_new = kalman_update(x_new, z[robot,:], P_new, np.diag(R[robot,:])) # get uncertainity in the new configuration
                   
                q_new = AIANode(p_new, x_new, P_new, u_new) # create new node of the graph

                tree.append_child(q_rand, q_new, np.linalg.det(P_new))
                    
    # Get the sequence of actions to follow
    path = tree.get_min_path(delta)
    return path, tree

   