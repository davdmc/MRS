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
    def __init__(self, size, period, A, B, C, q) -> None:
        self.size = size
        self.period = period
        self.A = A
        self.B = B
        self.C = C
        self.q = q
        self.map = np.zeros(size)
        self.map[:,:] = 0
        self.k = 0
        self.end_k = 1000
        self.previous_states = np.zeros((size[0], size[1], self.end_k))
        self.robot_x = 10
        self.robot_y = 10
        # size = 6
        self.sigma = matlab_style_gauss2D((13, 13), sigma=2.4)
        self.robot_action_half_size = int(self.sigma.shape[0]/2)
        self.fig = plt.figure()
        self.desired = 50
        self.errors = []
        #ani = FuncAnimation(self.fig, self.act_drawing)
        #self.run()

    def run(self, route_x, route_y, end_k):
        self.end_k = end_k
        for self.k in range(self.end_k):
            self.robot_x = self.robot_x + route_x[self.k]
            self.robot_y = self.robot_y + route_y[self.k]
            F = np.exp(self.A * self.period)
            G = (self.B/self.A) * (np.exp(self.A * self.period) - 1)
            self.previous_states[:,:,self.k] = self.map
            action_map = np.zeros(self.map.shape)
            coverage_local = self.map[self.robot_x-self.robot_action_half_size:self.robot_x+self.robot_action_half_size+1,  self.robot_y-self.robot_action_half_size:self.robot_y+self.robot_action_half_size+1]
            K = self.C*(np.sum(np.sum(self.B * self.sigma * (self.desired-coverage_local))))**(2*self.q-1)
            alpha = K*self.sigma
            action_map[self.robot_x-self.robot_action_half_size:self.robot_x+self.robot_action_half_size+1,  self.robot_y-self.robot_action_half_size:self.robot_y+self.robot_action_half_size+1]= alpha
            self.map = F * self.map + G * action_map
            self.draw_map()
            self.draw_error()

    def draw_map(self):
        plt.figure(1)
        plt.clf()
        plt.imshow(self.map, cmap='jet', vmin=0, vmax=100)
        plt.colorbar()
        plt.show(block=False)
        plt.pause(0.0001)
    
    def draw_error(self):
        plt.figure(2)
        plt.clf()
        self.errors.append(np.sum(np.sum(((np.zeros(self.size)+1)*self.desired - self.map)**2))/(100*100))
        plt.plot(self.errors)
        plt.show(block=False)
        plt.pause(0.0001)



if __name__ == '__main__':

    # my_environment = Environment([100,100], 1, -1/200, 1/400, 120000, 1)
    my_environment = Environment([100,100], 1, -1/200, 1/400, 120000, 1/2)


    end_k = 804
    route_x = []
    route_y = []

    for j in range(4):
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
    for j in range(100):
        route_x.append(0)
        route_y.append(0)
        
    my_environment.run(route_x, route_y, end_k)