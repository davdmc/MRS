import numpy as np

from graph import AIANode, AIATree
from utils import kalman_predict, kalman_update
import datetime


def sampling_based_active_information_acquisition(max_n, environment, delta_in):
    '''
        Sampling-based Active Information Filter
    '''
    path = -1
    min_node_cost = 10000
    delta = delta_in
    while path==-1:
        start = datetime.datetime.now()
        x_est = environment.x_est
        P_est = environment.P_est

        root = AIANode(environment.qi, x_est, P_est, np.zeros(environment.qi.shape))
        root.id = 0
        root.timestamp = 0
        root.cov = P_est * 100000
        root.cost = np.linalg.det(P_est)

        tree = AIATree(root)

        for n in range(max_n):
            vk_rand = tree.sample_fv() # Sample a k (corresponds to a position of 1 or more nodes with the same configuration)
            u_new = environment.sample_fu() # Sample an action
            p_new = np.reshape(vk_rand, np.shape(u_new)) + u_new # New position with that action

            if environment.free_space(p_new):
                for q_rand in tree.v_k[(vk_rand).tostring()]: # Apply it for all the nodes with that configuration
                    z, R = environment.get_measurements(p_new)
                    x_est, P_est = kalman_predict(q_rand.x_est, q_rand.cov, environment.Q)
                    x_new, P_new = kalman_update(x_est, z[0,:], P_est, np.diag(R[0,:])) # get uncertainity in the new configuration
                    for robot in range(1,environment.n_agents):
                        x_new, P_new = kalman_update(x_new, z[robot,:], P_new, np.diag(R[robot,:])) # get uncertainity in the new configuration
                    
                    q_new = AIANode(p_new, x_new, P_new, u_new) # create new node of the graph

                    tree.append_child(q_rand, q_new, np.linalg.det(P_new))
                        
        # Get the sequence of actions to follow
        path, min_node_cost = tree.get_min_path(delta)
        environment.cost_node.append(min_node_cost)
        np.save("output/cost_node", environment.cost_node)
        delta = delta * 2
        end = datetime.datetime.now()
    interval = end-start
    environment.time.append(interval.total_seconds())
    np.save("output/time", environment.time)
    return path, tree