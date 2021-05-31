## A solution to the Boids exercise
## Version: 20210517
## Author: Enrique Teruel (ET) eteruel@unizar.es
## License: CC-BY-SA

import math
from time import time, localtime, strftime
import numpy as np

# YOU MIGHT FIND USEFUL FUNCTIONS IN shapely, Point2D, AND gadgETs
from point2d import Point2D
from small_worl2d_config import visual, TS, RT
from small_worl2d import GoTo, Space, KPIdata, MoBody, Mobot, Soul

class Boid(Soul):

    def __init__(self,body,space,goodistance=1,T=TS): # ADD YOUR ARGUMENTS
        self.space=space
        self.goodistance=goodistance
        
        # YOUR BOID INIT CODE
        self.k_p = 0.3
        super().__init__(body,T)
        

    def update(self):
        b=self.body
        i=self.space.bodindex(b.name)
        # YOUR BOID UPDATE CODE
        neigh=self.space.nearby(i,3,math.pi,type(self.body))
        neigh.append(self.body)

        #neigh=self.space.bodies
        # Compute vectors to neightbors which allows to compute (attraction)
        # and repulsion. Also orientation differences.
        # Create structures for storing the data
        num_n = float(len(neigh))
        centroid_pos = Point2D(0.0,0.0)
        v_repulse = Point2D(0.0,0.0)
        v_bearing = Point2D(0.0,0.0) 

        for n in neigh:
            # Adjust centroid
            centroid_pos += Point2D(n.pos.x, n.pos.y)

            if(self.body.name != n.name):
                # Compute repulsive direction
                v_n = Point2D(float(self.body.pos.x - n.pos.x), float(self.body.pos.y - n.pos.y))
                if(v_n.r < self.goodistance):
                    rep_dist = (v_n.r - self.goodistance)
                    v_repulse += v_n * rep_dist * rep_dist

            # Compute avg velocity
            #bearing = Point2D(math.cos(n.th), math.sin(n.th))
            v_bearing += Point2D(math.cos(n.th) * n.v, math.sin(n.th) * n.v)

        v_bearing.x = v_bearing.x/num_n
        v_bearing.y = v_bearing.y/num_n
        centroid_pos.x = centroid_pos.x/num_n
        centroid_pos.y = centroid_pos.y/num_n
        v_centroid = Point2D(centroid_pos.x-float(self.body.pos.x),centroid_pos.y-float(self.body.pos.y))

        if(float(self.body.name[1:]) % 4 == 0):
            v_bearing = Point2D(1,0)
        
        v_steering = v_centroid + v_repulse + v_bearing
        v_steering = v_steering / v_steering.r
        self.steer = v_steering
        s_cos = math.cos(self.body.th)
        s_sin = math.sin(self.body.th)
        s_x =  s_cos * v_steering.x + s_sin * v_steering.y
        s_y =  - s_sin * v_steering.x + s_cos * v_steering.y
        u = max(0, s_x) * self.body.v_max
        w = self.k_p * (math.atan2(v_steering.y,v_steering.x) - self.body.th)
        self.body.cmd_vel(u, w, 0.0)
        self.vertices=[(self.body.pos.x,self.body.pos.y),(self.body.pos.x+v_steering.x,self.body.pos.y+v_steering.y)]
        super().update()


## MAIN

R=1.8 # so goodistance=1 is little less than half R
name='Boids_R'+str(R)+'_'+strftime("%Y%m%d%H%M", localtime())
s=Space(name,T=RT,R=R,limits='')
#p=KPIdata(name,5,TS)
N=50
s.spawn_bodies(N,room=[(-4,-4),(4,-4),(4,4),(-4,4)])
for b in s.bodies:
    if isinstance(b,Mobot):
        Boid(b,s,R,0.1*TS) # ADD YOUR ARGUMENTS
#KPI=[1,1,np.sqrt(N),1,0]
end=False
while not end:
    for b in s.bodies:
        if isinstance(b,MoBody):
            b.update()
    s.update_dist()
    s.remobodies(s.collisions())
    s.update_conn()
    if visual and time()>s.t0+(s.updates+1)*s.T:
        s.flash([s.bodies[23]])
        s.redraw()
        
    #if time()>p.t0+(p.updates+1)*p.T:
        # YOUR KPI CALCULATIONS
        #p.update(KPI)
    end= s.has_been_closed() #or KPI[1]==0 or p.updates>300  5 realtime min
else:
    s.close()
    #p.close()
