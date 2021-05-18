import time

import numpy as np

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from numpy.lib.twodim_base import mask_indices
import scipy.stats as st
import math

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

def gauss_der(shape=(3,3),sigma=0.5):
    m,n = [(ss-1.)/2. for ss in shape]
    y,x = np.ogrid[-m:m+1,-n:n+1]
    h = np.sqrt(x*x + y*y)*np.exp( -(x*x + y*y) / (2.*sigma*sigma) )
    h[ h < np.finfo(h.dtype).eps*h.max() ] = 0
    sumh = h.sum()
    if sumh != 0:
        h /= sumh
    return h/2

def angles(shape=(3,3)):
    result = np.zeros(shape)
    for x in range(shape[0]):
        for y in range(shape[1]):
            result[x,y] = math.atan2(y, x)
    return result

class Environment:
    def __init__(self, size, period, A, B, C, q) -> None:
        self.size = size
        self.period = period
        self.A = A
        self.B = B
        self.C = C
        self.q = q
        self.map = np.zeros(size)
        self.map[0:3, :] = np.ones((3,size[1]))*100
        self.map[:, 0:3] = np.ones((size[0], 3))*100
        self.map[-3:, :] = np.ones((3,size[1]))*100
        self.map[:, -3:] = np.ones((size[0], 3))*100
        self.weighted_map = matlab_style_gauss2D(size, sigma=20)
        self.weighted_map = self.weighted_map/np.max(self.weighted_map)
        self.weighted_map[0:3, :] = np.ones((3,size[1]))*1
        self.weighted_map[:, 0:3] = np.ones((size[0], 3))*1
        self.weighted_map[-3:, :] = np.ones((3,size[1]))*1
        self.weighted_map[:, -3:] = np.ones((size[0], 3))*1
        plt.imshow(self.weighted_map, cmap='jet')
        plt.colorbar()
        # self.weighted_map = np.ones(size)
        # self.map[:,:] = 0
        self.k = 0
        self.end_k = 1000
        self.previous_states = np.zeros((size[0], size[1], self.end_k))
        self.robot_x = [10, 10, 25, 40, 40]
        self.robot_y = [10, 40, 25, 10, 40]
        self.desired = 50
        size = 6
        self.sigma = matlab_style_gauss2D((7,7), sigma=1.5)
        self.derivative = gauss_der((7,7), sigma=1.5)
        # self.derivative_x = np.array([[1,4,5,0,-5,-4,-1],
        #                     [6,24,30,0,-30,-24,-6],
        #                     [15,60,75,0,-75,-60,-15],
        #                     [20,80,100,0,-100,-80,-20],
        #                     [15,60,75,0,-75,-60,-15],
        #                     [6,24,30,0,-30,-24,-6],
        #                     [1,4,5,0,-5,-4,1]])
        # self.derivative_x = np.array([[0,0,1,1,1,0,0],
        #                               [0,1,3,3,3,1,0],
        #                               [1,3,0,-7,0,3,1],
        #                               [1,3,-7,-24,-7,3,1],
        #                               [1,3,0,-7,0,3,1],
        #                               [0,1,3,3,3,1,0],
        #                               [0,0,1,1,1,0,0]])
        # self.position_neigh = np.array([range(-3,4),]*7)
        self.position_neigh = angles((7,7))
        # self.position_neigh = np.array([[3,2,1,0,1,2,3],]*7)
        self.robot_action_half_size = int(self.sigma.shape[0]/2)
        self.dist_matrix = self.distance_matrix_calc()
        self.errors = []
        self.fig = plt.figure()
        #ani = FuncAnimation(self.fig, self.act_drawing)
        #self.run()

    def distance_matrix_calc(self):
        dis_mat = np.zeros(np.shape(self.sigma))
        for i in range(np.shape(self.sigma)[0]):
            for j in range(np.shape(self.sigma)[1]):
                dis_mat[i,j] = math.sqrt((i-self.robot_action_half_size)**2 + (j-self.robot_action_half_size)**2)
        dis_mat[self.robot_action_half_size, self.robot_action_half_size] = 1
        return dis_mat

    def run(self, route_x, route_y, end_k):
        self.end_k = end_k
        for self.k in range(self.end_k):
            for i in range(len(self.robot_x)):
                [ux,uy] = self.local_motion(self.robot_x[i], self.robot_y[i])
                # self.robot_x = self.robot_x + route_x[self.k]
                # self.robot_y = self.robot_y + route_y[self.k]
                self.robot_x[i] = min(max(self.robot_x[i] + ux, 3), 46)
                self.robot_y[i] = min(max(self.robot_y[i] + uy, 3), 46)
                F = np.exp(self.A * self.period)
                G = (self.B/self.A) * (np.exp(self.A * self.period) - 1)
                self.previous_states[:,:,self.k] = self.map
                action_map = np.zeros(self.map.shape)
                print(self.robot_x[i], self.robot_y[i], self.robot_action_half_size)
                print(action_map.shape, self.sigma.shape)
                weighted_local = self.weighted_map[self.robot_x[i]-self.robot_action_half_size:self.robot_x[i]+self.robot_action_half_size+1,  self.robot_y[i]-self.robot_action_half_size:self.robot_y[i]+self.robot_action_half_size+1]
                coverage_local = self.map[self.robot_x[i]-self.robot_action_half_size:self.robot_x[i]+self.robot_action_half_size+1,  self.robot_y[i]-self.robot_action_half_size:self.robot_y[i]+self.robot_action_half_size+1]
                K = self.C*(np.sum(np.sum(self.B * self.sigma * weighted_local * (self.desired-coverage_local))))**(2*self.q-1)
                alpha = K*self.sigma
                action_map[self.robot_x[i]-self.robot_action_half_size:self.robot_x[i]+self.robot_action_half_size+1,  self.robot_y[i]-self.robot_action_half_size:self.robot_y[i]+self.robot_action_half_size+1]= alpha
                self.map = F * self.map + G * action_map
                self.map[0:3, :] = np.ones((3,self.size[1]))*100
                self.map[:, 0:3] = np.ones((self.size[0], 3))*100
                self.map[-3:, :] = np.ones((3,self.size[1]))*100
                self.map[:, -3:] = np.ones((self.size[0], 3))*100
            self.draw_map()
            self.draw_error()

    def draw_map(self):
        plt.figure(2)
        plt.clf()
        plt.imshow(self.map, cmap='jet', vmin=0, vmax=100)
        plt.colorbar()
        plt.show(block=False)
        plt.pause(0.001)

    def draw_error(self):
        plt.figure(3)
        plt.clf()
        self.errors.append(np.sum(np.sum(((((np.zeros(self.size)+1)*self.desired - self.map)**2)*self.weighted_map)[3:-3, 3:-3]))/(47*47))
        plt.plot(self.errors)
        plt.show(block=False)
        plt.pause(0.001)

    def local_motion(self, x_robot, y_robot):
        weighted_local = self.weighted_map[x_robot-self.robot_action_half_size:x_robot+self.robot_action_half_size+1,  y_robot-self.robot_action_half_size:y_robot+self.robot_action_half_size+1]
        coverage_local = self.map[x_robot-self.robot_action_half_size:x_robot+self.robot_action_half_size+1,  y_robot-self.robot_action_half_size:y_robot+self.robot_action_half_size+1]
               
        first_integral = np.sum(np.sum(self.B * self.sigma * weighted_local * (self.desired-coverage_local)))
        second_integral = np.sum(np.sum(self.B * weighted_local * self.derivative * (self.position_neigh/self.dist_matrix) * (self.desired-coverage_local)))
        # second_integral_y = np.sum(np.sum(self.B * weighted_local * np.transpose(self.derivative) * (np.transpose(self.position_neigh)/self.dist_matrix) * (self.desired-coverage_local)))
        u = -4*self.q*self.C*(first_integral**(2*self.q-1)) * second_integral
        ux = math.cos(u)*3
        uy = math.sin(u)*3
        # uy = -4*self.q*self.C*(first_integral**(2*self.q-1)) * second_integral_y
        # print(coverage_local)
        print(ux, uy)
        # return np.array([min(max(np.round(ux), -1), 1),min(max(np.round(uy), -1), 1)], dtype=np.int32)
        # return np.array([np.sign(ux),np.sign(uy)], dtype=np.int32)
        return np.array([np.round(ux), np.round(uy)], dtype=np.int32)

if __name__ == '__main__':

    my_environment = Environment([50,50], 1, -1/200, 1/100, 150000, 1)
    # my_environment = Environment([100,100], 0.01, -0.5, 0.00005, 12, 0.5)


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