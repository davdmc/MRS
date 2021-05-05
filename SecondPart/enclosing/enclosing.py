import numpy as np

class Enclosing:
    def __init__(self, q_x, q_y, ci):
        # Robots qi positions, qn is assumed to be the target
        self.q = np.array([q_x,q_y])
        self.num_pos = self.q.shape[0]

        # Inter-robot relative position vectors ci R(N*N*2)
        self.ci = ci

        self.iters = 1000
        self.K_c = 10
        self.delta_t = 0.1
    
    def run(self):

        for i in range(self.iters):
            Q = self.compute_inter_positions(self.q)
            C = self.compute_inter_positions(self.ci)
            A = C.t @ Q
            U, S, V_t = np.linalg.svd(A)
            d = np.sign(np.linalg.det(V_t.t @ U.t))
            D = np.array([[1,0],[0,d]])
            R = V_t.t @ D @ U.t
            q_Ni = Q[0:self.num_pos:]
            c_Ni = C[0:self.num_pos:]
            q_dot = self.K_c * (q_Ni - R @ c_Ni)
            self.q += q_dot * self.delta_t

       
    def compute_inter_positions(self, positions):
    
        qij = np.zeros(self.num_pos, self.num_pos, 2)
        # Compute inter-robot relative positions
        for i in range(self.num_pos):
            for j in range(self.num_pos):
                qij[i+j, 0] = self.q[j,0] - self.q_x[i,0]
                qij[i+j, 1] = self.q[j,1] - self.q[i,1]

        return qij

    def plot_robots(self):

        #TODO: MATPLOT