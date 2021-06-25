## A solution to the Boids exercise
## Version: 20210517
## Author: Enrique Teruel (ET) eteruel@unizar.es
## License: CC-BY-SA

import math
from time import time, localtime, strftime
import numpy as np
from tqdm import tqdm

# YOU MIGHT FIND USEFUL FUNCTIONS IN shapely, Point2D, AND gadgETs
from point2d import Point2D
from small_worl2d_config import visual, TS, RT
from small_worl2d import GoTo, Space, KPIdata, MoBody, Mobot, Soul
from gadgETs import get_all_cc

class Boid(Soul):

    def __init__(self,body,space,goodistance=1,T=TS): # ADD YOUR ARGUMENTS
        self.space=space
        self.goodistance=goodistance
        
        # YOUR BOID INIT CODE
        self.k_p = 0.1
        self.dist_err = 0.0
        self.cent_mod = 0.0
        self.rep_mod = 0.0
        self.bearing_mod = 0.0
        self.direction_mod = 0.0
        super().__init__(body,T)

        

    def update(self):
        b=self.body
        i=self.space.bodindex(b.name)
        # YOUR BOID UPDATE CODE
        neigh=self.space.nearby(i,3,math.pi,type(self.body))
        # Alternative taking self into account (resist to other's influence): neigh.append(self.body)

        # Compute steering vector components
        # Create structures for storing the data
        num_n = float(len(neigh))
        if(num_n == 0):
            return
        centroid_pos = Point2D(0.0,0.0)
        v_repulse = Point2D(0.0,0.0)
        v_bearing = Point2D(0.0,0.0)
        # KPI[3] 
        self.dist_err = 0.0
        for n in neigh:
            # Centroid atraction
            centroid_pos += Point2D(n.pos.x, n.pos.y)

            # Repulsion vector
            if(self.body.name != n.name):
                # Compute repulsive direction
                v_n = Point2D(float(self.body.pos.x - n.pos.x), float(self.body.pos.y - n.pos.y))
                # KPI[3]: err_goodistance / goodistance
                self.dist_err += (v_n.r - self.goodistance) / self.goodistance
                if(v_n.r < self.goodistance):
                    rep_dist = (1/v_n.r - 1/self.goodistance)
                    v_repulse += v_n * (rep_dist * rep_dist)
                    
            # Compute avg velocity orientation
            v_bearing += Point2D(math.cos(n.th), math.sin(n.th))

        # Compute actual centroid
        centroid_pos = centroid_pos/num_n
        # Vector from body to centroid
        v_centroid = Point2D(centroid_pos.x-float(self.body.pos.x),centroid_pos.y-float(self.body.pos.y))
        # KPI[3]: squared avg of error (to be averaged among all robots later)
        self.dist_err /= num_n
        self.dist_err = self.dist_err * self.dist_err
        
        # Average bearing
        v_bearing = v_bearing/num_n
        
        # Add informed direction to some of the robots
        if(float(self.body.name[1:]) % 10 == 0):
            v_direction = Point2D(1.0, 0.0)
        else:
            v_direction = Point2D(0.0, 0.0)

        # Combine steering vector components
        v_steering = v_centroid + v_repulse + v_bearing + v_direction
        self.cent_mod = (v_centroid.r / (v_centroid.r + v_repulse.r + v_bearing.r + v_direction.r))
        self.rep_mod = (v_repulse.r / (v_centroid.r + v_repulse.r + v_bearing.r + v_direction.r))
        self.bearing_mod = (v_bearing.r / (v_centroid.r + v_repulse.r + v_bearing.r + v_direction.r))
        self.direction_mod = (v_direction.r / (v_centroid.r + v_repulse.r + v_bearing.r + v_direction.r))
        v_steering = v_steering / v_steering.r
        self.steer = v_steering
        
        # Transform from global coordinates to local coordinates by rotating steering vector: 
        # v_w * R_w_b (with angle of R_w_b = self.th)
        s_cos = math.cos(self.body.th)
        s_sin = math.sin(self.body.th)
        s_x =  s_cos * v_steering.x + s_sin * v_steering.y
        s_y =  - s_sin * v_steering.x + s_cos * v_steering.y

        u = max(0, s_x) * self.body.v_max
        w = self.k_p * (math.atan2(v_steering.y,v_steering.x) - self.body.th)
        # Alternative (fraction of tangential component): w = self.k_p * s_y

        self.body.cmd_vel(u, w, 0.0)

        # Draw the steering vector
        self.vertices=[(self.body.pos.x,self.body.pos.y),(self.body.pos.x+v_steering.x/2,self.body.pos.y+v_steering.y/2)]
        super().update()


## MAIN

R=1.8 # so goodistance=1 is little less than half R
name='Boids_R'+str(R)+'_'+strftime("%Y%m%d%H%M", localtime())
s=Space(name,T=RT,R=R,limits='')
# KPI datastruct:
# [0]: Fraction of N still alive (1 means no collision)
# [1]: Fraction of still alive Mobot’s in the largest connected component (1 means all connected)
# [2]: Cohesion radius (maximum distance to centroid) divided by sqrt(N) * ”good distance”
# [3]: Average square (error to “good distance” divided by “good distance”)
# [4]: Effect of steering vector components
# [5]: ...
# [6]: ...
# [7]: ...

#p=KPIdata(name,7,TS)
N=50
period = 0.1
total_time = 5 * 60
steps = int(total_time/period)

s.spawn_bodies(N,room=[(-4,-4),(4,-4),(4,4),(-4,4)])
for b in s.bodies:
    if isinstance(b,Mobot):
        Boid(b,s,R,0.1*TS) # ADD YOUR ARGUMENTS

data_gather = []
KPI=[0,0,0,0,0,0,0,0]

for i in tqdm(range(steps+1)):
    # KPI[2]
    centroid_pos = Point2D(0.0,0.0)
    for b in s.bodies:
        if isinstance(b,MoBody):
            b.update()

    s.update_dist()
    s.remobodies(s.collisions())
    s.update_conn()
    if visual and time()>s.t0+(s.updates+1)*s.T:
        s.redraw()
    
    KPI=[0,0,0,0,0,0,0,0]

    # First loop
    for b in s.bodies:
        if isinstance(b,Mobot):
            # KPI[0]
            KPI[0] += 1
            # KPI[2]
            centroid_pos += Point2D(b.pos.x, b.pos.y)

            soul = b.souls[0]
            # KPI[3]
            KPI[3] += soul.dist_err
            # KPI[4,5,6,7]
            KPI[4] += soul.cent_mod 
            KPI[5] += soul.rep_mod 
            KPI[6] += soul.bearing_mod 
            KPI[7] += soul.direction_mod

    centroid_pos = centroid_pos/KPI[0]
    
    # Second loop: only to check the greatest dist to centroid
    for b in s.bodies:
        if isinstance(b,Mobot):
            v_b_c = centroid_pos - Point2D(b.pos.x, b.pos.y)
            if v_b_c.r > KPI[2]:
                KPI[2] = v_b_c.r

    # KPI[1]
    connected = get_all_cc(s.graph(Mobot))
    greater_size = 0
    #greater_idx = 0
    for idx, connected_comp in enumerate(connected):
        if len(connected_comp) > greater_size:
            #greater_idx = idx
            greater_size = len(connected_comp)
    KPI[1] = greater_size / KPI[0]

    # KPI[2]
    KPI[2] /= (math.sqrt(N) * R)
    # KPI[3]
    KPI[3] /= KPI[0]
    # KPI[4,5,6,7]
    KPI[4] /= KPI[0]
    KPI[5] /= KPI[0]
    KPI[6] /= KPI[0]
    KPI[7] /= KPI[0]
    # KPI[0] IMPORTANT TO BE THE LAST (before it was number of alive robots)
    KPI[0] /= N
    data_gather.append(KPI)
    
    #p.update(KPI)

s.close()
#p.close()
data_np = np.array(data_gather)
np.savetxt(name+'KPIs.csv', data_np, delimiter=',', fmt='%s')