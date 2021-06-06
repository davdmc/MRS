import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


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

    def update(self):
        self.hist_qi.append(self.qi)
        noise_agents = np.random.randn(self.qi.shape[0], self.qi.shape[1]) * self.sigma_agents
        self.qi += self.u_agents * self.t + noise_agents
    
        self.hist_xi.append(self.xi)
        noise_targets = np.random.randn(self.xi.shape[0], self.xi.shape[1]) * self.sigma_targets
        self.xi += self.u_targets * self.t + noise_targets

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