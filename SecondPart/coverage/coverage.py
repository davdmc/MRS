import time

import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from numpy.lib.twodim_base import mask_indices
import scipy.stats as st

def matlab_style_gauss2D(shape=(3,3),sigma=0.5):
    """
    2D gaussian mask - should give the same result as MATLAB's
    fspecial('gaussian',[shape],[sigma])
    """
    m,n = [(ss-1.)/2. for ss in shape]
    y,x = np.ogrid[-m:m+1,-n:n+1]
    h = np.exp( -(x*x + y*y) / (2.*sigma*sigma) )
    h[ h < np.finfo(h.dtype).eps*h.max() ] = 0
    sumh = h.sum()
    if sumh != 0:
        h /= sumh
    return h

class Environment:
    def __init__(self, size, period, A, B) -> None:
        self.size = size
        self.period = period
        self.A = A
        self.B = B
        self.map = np.zeros(size)
        self.map[:,:] = 0
        self.k = 0
        self.end_k = 1000
        self.previous_states = np.zeros((size[0], size[1], self.end_k))
        self.robot_x = 10
        self.robot_y = 10
        size = 6
        self.robot_action = matlab_style_gauss2D((12,12), sigma=2.4)
        self.robot_action_half_size = int(self.robot_action.shape[0]/2)
        self.fig = plt.figure()
        #ani = FuncAnimation(self.fig, self.act_drawing)
        #self.run()

    def run(self, route_x, route_y, end_k):
        self.end_k = end_k
        for self.k in range(self.end_k):
            self.robot_x = self.robot_x + route_x[self.k]
            self.robot_y = self.robot_y + route_y[self.k]
            F = np.exp(self.A * self.period)
            G = (self.A/self.B) * (np.exp(self.A * self.period) - 1)
            self.previous_states[:,:,self.k] = self.map
            action_map = np.zeros(self.map.shape)
            action_map[max(0,self.robot_x-self.robot_action_half_size):min(self.map.shape[0], self.robot_x+self.robot_action_half_size), 
                        max(0,self.robot_y-self.robot_action_half_size):min(self.map.shape[1],self.robot_y+self.robot_action_half_size)] = self.robot_action
            self.map = F * self.map + G * action_map
            self.draw_map()

    def draw_map(self):
        plt.clf()
        plt.imshow(self.map, cmap='jet', vmin=0, vmax=100)
        plt.colorbar()
        plt.show(block=False)
        plt.pause(0.001)


if __name__ == '__main__':

    my_environment = Environment([100,100], 0.01, -1, 0.000025)

    end_k = 1760
    route_x = []
    route_y = []

    for j in range(10):
        for i in range(80):
            route_x.append(1)
            route_y.append(0)
        
        for i in range(8):
            route_x.append(0)
            route_y.append(1)

        for i in range(80):
            route_x.append(-1)
            route_y.append(0)
        
        for i in range(8):
            route_x.append(0)
            route_y.append(1)
        
    my_environment.run(route_x, route_y, end_k)