import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from numpy import random
from numpy.linalg.linalg import det

class q_class:
    def __init__(self, p, sigma, id, t):
        self.p = p
        self.sigma = sigma
        self.id = id
        self.t = t
        self.cost = 0

    def update_cost(self, previous_cost):
        self.cost = previous_cost + np.linalg.det(self.sigma)

class Environment:
    def __init__(self, width, height, t, visualize=True):
        
        self.width = width
        self.height = height
        self.t = t

        # ROW: agent/target, COL: pos x pos y
        self.qi = np.zeros((0,2))
        self.sigma_agents = np.zeros((0,2))
        self.n_agents = 0
        self.u_agents = np.zeros((0,2))
        self.hist_qi = []

        self.xi = np.zeros((0,2))  
        self.sigma_targets = np.zeros((0,2))
        self.n_targets = 0
        self.u_targets = np.zeros((0,2))
        self.hist_xi = []

        self.id_nodes_graph = 0
        self.delta = 0.0000018 #proposed by the paper
        self.sigma_estimated = []
        self.x_estimated = []

        if visualize:
            # Configure plot
            print("Goes in")
            plt.plot(0,0)
            self.ani = FuncAnimation(plt.gcf(), self.plot_env, interval=self.t*1000)
            
    def add_agent(self, x, y, sigma_x, sigma_y):
        self.qi = np.vstack((self.qi, np.array([x,y])))
        self.u_agents = np.vstack((self.u_agents, np.array([0,0])))
        self.sigma_agents = np.vstack((self.sigma_agents, np.array([sigma_x, sigma_y])))
        self.n_agents += 1
    
    def add_target(self, x, y, sigma_x, sigma_y):
        self.xi = np.vstack((self.xi, np.array([x,y])))
        self.u_targets = np.vstack((self.u_targets, (np.random.random((1,2))-0.5)*5))
        self.sigma_targets = np.vstack((self.sigma_targets, np.array([sigma_x, sigma_y])))
        self.n_targets += 1
        self.x_estimated = np.vstack((np.array(self.x_estimated), x,y))
        sigma_aux = np.identity(len(self.sigma_estimated)+2)
        np.fill_diagonal(sigma_aux, np.hstack(np.diag(sigma_aux), sigma_x, sigma_y))
        self.sigma_estimated = sigma_aux

    def apply_kalman_filter(self, pos_q, sigma_in):
        # Use a Kalman filter to obtain measurement and covariance of the 
        # measurements of the targets. We assume that the robot possition is known
        # and that every robot can measure all the targets, but the covariance of
        # the measurement is proportional to the distance to the target.
        sigma = sigma_in
        x = self.x_estimated
        for agent in len(self.n_agents):
            measurements_sigma = np.identity(self.n_targets*2)
            measurements_x = np.zeros(self.n_targets*2, 1)
            for target in range(self.n_targets):
                difx = pos_q[agent, 0] - self.xi[target, 0]
                dify = pos_q[agent, 1] - self.xi[target, 1]
                measurements_sigma[2*target, 2*target] = 0.01 * np.sqrt(difx**2 + dify**2)
                measurements_sigma[2*target + 1, 2*target+1] = 0.01 * np.sqrt(difx**2 + dify**2)
                measurements_x[2*target] = np.random.randn()*measurements_sigma[2*target, 2*target]
                measurements_x[2*target+1] = np.random.randn()*measurements_sigma[2*target+1, 2*target+1]
            y = measurements_x - x
            S = sigma + measurements_sigma
            K = sigma@measurements_sigma
            x = x + K*y
            sigma = (np.identity(len(K))-K) * self.sigma_estimated
        return x, sigma

    def update(self):
        self.hist_qi.append(self.qi)
        noise_agents = np.random.randn(self.qi.shape[0], self.qi.shape[1]) * self.sigma_agents
        self.qi += self.u_agents * self.t + noise_agents
    
        self.hist_xi.append(self.xi)
        noise_targets = np.random.randn(self.xi.shape[0], self.xi.shape[1]) * self.sigma_targets
        self.xi += self.u_targets * self.t + noise_targets

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
        sampling_vector = []
        for i in range(len(V)):
            found = False
            for q in V(i):
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
        # use an infinite sensor range, and the preferred final u will be always selected
        # using the covariance of the measurements. Thus, the sampling of u is completely
        # random.
        u_possibilities = [0, 0.5, -0.5]
        return random.choice(u_possibilities, (np.shape(self.u_agents)))



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
        q_0 = q_class(self.qi, self.sigma_estimated, self.id_nodes_graph, q_rand.t + 1)
        q_0.update_cost(0)
        V_total = [q_0]
        Vk = [[q_0]]
        epsilon = []
        K = 0
        Xg = []
        N = 10
        max_t = 0
        for n in range(N):
            Vkrand = self.sample_fv(Vk, max_t)
            u_new = self.sample_fu()
            p_new = q_0.p + u_new * self.t
            if self.free_space(p_new):
                for q_rand in Vk[Vkrand]:
                    x, sigma_new = self.apply_kalman_filter(q_rand.p, q_rand.sigma)
                    q_new = q_class(p_new, sigma_new, self.id_nodes_graph, q_rand.t + 1)
                    self.id_nodes_graph = self.id_nodes_graph+1
                    q_new.update_cost(q_rand.cost)
                    max_t = max(max_t, q_new.t)
                    V_total.append(q_new)
                    epsilon.append([q_rand.id, q_new.id])
                    if same_configuration(q_new, Vk):
                        Vk[Vkrand].append(q_new)
                    else:
                        K = K+1
                        Vk.append([q_new])
                    if np.linalg.det(q_new.sigma) <= self.delta:
                        Xg.append(q_new)
        path = find_recover_path(q_end)


