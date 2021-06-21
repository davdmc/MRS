## A Demo of SmallWorl2D
## Version: 1.0.20210514
## Author: Enrique Teruel (ET) eteruel@unizar.es
## License: CC-BY-SA

from time import time, localtime, strftime
from random import uniform
import random
import numpy as np
from matplotlib.animation import FFMpegWriter # requires having ffmpeg installed, from https://ffmpeg.org/

from small_worl2d_config import visual, W, H, TS, RT # configuration parameters in a separate file
from small_worl2d import Space, KPIdata, Nest, Mobot, Soul, GoTo, Knowledge

class Nestxists(Knowledge):
    """ A silly specialization of Knowledge, actually it is just the basic Knowledge
        
        Typically, here would be defined the specific variables and functions required
    """

    def __init__(self,body,state):
        self.nests_known = [-1,-1,-1,-1]
        super().__init__(body,state)
    
    def tell_nests(self):
        return self.nests_known
    
    def add_nest(self, quadrant, size):
        if quadrant == self.nests_known[0]:
            if size > self.nests_known[1]:
                self.nests_known[1] = size
        elif quadrant == self.nests_known[2]:
            if size > self.nests_known[3]:
                self.nests_known[3] = size
        elif self.nests_known[0] == -1:
            self.nests_known[1] = size
            self.nests_known[0] = quadrant
        elif self.nests_known[2] == -1:
            self.nests_known[3] = size
            self.nests_known[2] = quadrant    

class Demo(Soul):
    """ A Soul that wanders differently if it knows ThereAreNests or Nothing """

    def __init__(self,body,space,T=TS,r=1,rng=np.pi):
        self.space=space
        self.r=r
        self.rng=rng # might be smaller than pi, but not too much
        GoTo(body,0,Kp=0.2,tol=0.01,nw='wander',p=0.1) # requires a GoTo soul in the same body (to go nowhere)
        self.GoTo=body.souls[-1] # this way it knows how to call it
        Nestxists(body,('KnowNothing')) # this Soul needs a Mind in its Body to work
        self.cicles_time = 0
        super().__init__(body,T)
        self.current_quadrant = self.getQuadrant()
        # self.body.set_vel_max(self.body.v_max*2,self.body.w_max)
    
    def getQuadrant(self):
        if self.body.pos.x < 0 and self.body.pos.y < 0:
            return 0
        elif self.body.pos.x < 0 and self.body.pos.y >= 0:
            return 1
        elif self.body.pos.x >= 0 and self.body.pos.y < 0:
            return 2
        else:
            return 3
    
    def getPointQuadrant(self):
        if self.current_quadrant == 0:
            return (-W/2, -H/2)
        elif self.current_quadrant == 1:
            return (-W/2, H/2)
        elif self.current_quadrant == 2:
            return (W/2, -H/2)
        else:
            return (W/2, H/2)
    
    def update_nests(self):
        if self.body.knows.tell_nests()[0] == -1 and self.body.knows.tell_nests()[2] == -1:
            self.body.knows.set_state('KnowNothing')
        elif self.body.knows.tell_nests()[0] != -1 and self.body.knows.tell_nests()[2] != -1:
            self.body.knows.set_state('KnowTwoNest')
        else:
            self.body.knows.set_state('KnowOneNest')
            if self.current_quadrant == self.body.knows.tell_nests()[0] or self.current_quadrant == self.body.knows.tell_nests()[2]:
                aux = [0,1,2,3]
                aux.remove(max(self.body.knows.tell_nests()[0], self.body.knows.tell_nests()[2]))
                self.current_quadrant = random.choice(aux)
                self.cicles_time = 0

    
    def goToNestSelection(self):
        if self.body.knows.tell_nests()[1] == -1:
            return self.body.knows.tell_nests()[0]
        elif self.body.knows.tell_nests()[3] == -1:
            return self.body.knows.tell_nests()[2]
        elif self.body.knows.tell_nests()[1] >= self.body.knows.tell_nests()[3]:
            return self.body.knows.tell_nests()[0]
        else:
            return self.body.knows.tell_nests()[2]

    def update(self):
        i=self.space.bodindex(self.body.name)
        if self.body.knows.tell_state()=='KnowNothing':
            if uniform(0,1) < self.cicles_time/50000:
                aux = [0,1,2,3]
                aux.remove(self.current_quadrant)
                self.current_quadrant = random.choice(aux)
                self.cicles_time = 0
            self.body.fc = 'k'
            nest=self.space.nearest(i,self.r,self.rng,Nest)
            if nest!=None:
                self.GoTo.cmd_set(self.space.nearestpoint(i,nest)) # let's pay it a visit
            if len(self.space.incontact(i,Nest))>0:
                self.body.knows.add_nest(self.getQuadrant(), -1)
                self.GoTo.nw = 'wander'
            neigh=self.space.nearby(i,self.r,self.rng,type(self.body))
            for n in neigh:
                nests = n.knows.tell_nests()
                self.body.knows.add_nest(nests[0], nests[1])
                self.body.knows.add_nest(nests[2], nests[3])
            self.update_nests()
        elif self.body.knows.tell_state()=='KnowOneNest':
            if uniform(0,1) < self.cicles_time/50000:
                aux = [0,1,2,3]
                aux.remove(self.current_quadrant)
                if max(self.body.knows.tell_nests()[0], self.body.knows.tell_nests()[2]) != self.current_quadrant:
                    aux.remove(max(self.body.knows.tell_nests()[0], self.body.knows.tell_nests()[2]))
                self.current_quadrant = random.choice(aux)
                self.cicles_time = 0
            self.body.fc = 'r'
            i=self.space.bodindex(self.body.name)
            nest=self.space.nearest(i,self.r,self.rng,Nest)
            if nest!=None:
                self.GoTo.cmd_set(self.space.nearestpoint(i,nest)) # let's pay it a visit
            if len(self.space.incontact(i,Nest))>0:
                self.body.knows.add_nest(self.getQuadrant(), -1)
                self.GoTo.nw = 'wander'
            neigh=self.space.nearby(i,self.r,self.rng,type(self.body))
            for n in neigh:
                nests = n.knows.tell_nests()
                self.body.knows.add_nest(nests[0], nests[1])
                self.body.knows.add_nest(nests[2], nests[3])
            self.update_nests()
        elif self.body.knows.tell_state()=='KnowTwoNest':
            self.body.fc = 'g'
            self.GoTo.nw = 'wander'
            self.body.knows.set_state('GoToNest')
            self.cicles_time = 0
            self.current_quadrant = self.goToNestSelection()
            print("quadrant selected to go: ", self.current_quadrant)
        elif self.body.knows.tell_state()=='GoToNest':
            self.GoTo.nw = "wander"
            if uniform(0,1) < self.cicles_time/70000:
                self.GoTo.nw = "wander"
                self.body.knows.set_state('GoToNest')
                if self.body.knows.tell_nests()[0] == self.current_quadrant:
                    self.current_quadrant = self.body.knows.tell_nests()[2]
                else:
                    self.current_quadrant = self.body.knows.tell_nests()[0]
                self.cicles_time = 0
            self.body.fc = 'b'
            i=self.space.bodindex(self.body.name)
            nest=self.space.nearest(i,self.r,self.rng,Nest)
            if nest!=None:
                self.GoTo.cmd_set(self.space.nearestpoint(i,nest)) # let's pay it a visit
            if len(self.space.incontact(i,Nest))>0:
                self.cicles_time = 0
                self.body.knows.set_state('RestInNest')
                self.GoTo.nw = "zigzag"
                self.cicles_count = 1
            neigh=self.space.nearby(i,self.r,self.rng,type(self.body))
            for n in neigh:
                nests = n.knows.tell_nests()
                self.body.knows.add_nest(nests[0], nests[1])
                self.body.knows.add_nest(nests[2], nests[3])
        elif self.body.knows.tell_state()=='RestInNest':
            if self.body.knows.tell_nests()[1] == -1 or self.body.knows.tell_nests()[3] == -1:
                if uniform(0,1) < self.cicles_time/1000:
                    self.GoTo.nw = "wander"
                    self.body.knows.set_state('GoToNest')
                    if self.body.knows.tell_nests()[0] == self.current_quadrant:
                        self.current_quadrant = self.body.knows.tell_nests()[2]
                    else:
                        self.current_quadrant = self.body.knows.tell_nests()[0]
                    self.cicles_time = 0
            elif ((self.body.knows.tell_nests()[0] == self.current_quadrant and self.body.knows.tell_nests()[1] <= self.body.knows.tell_nests()[3]) or
            (self.body.knows.tell_nests()[2] == self.current_quadrant and self.body.knows.tell_nests()[3] <= self.body.knows.tell_nests()[1])):
                # Check if the current nest has bigger size, check the number of mobots
                if uniform(0,1) < self.cicles_time/1000:
                    self.GoTo.nw = "wander"
                    self.body.knows.set_state('GoToNest')
                    if self.body.knows.tell_nests()[0] == self.current_quadrant:
                        self.current_quadrant = self.body.knows.tell_nests()[2]
                    else:
                        self.current_quadrant = self.body.knows.tell_nests()[0]
                    self.cicles_time = 0
            else: #robot is in the expected biggest nest
                if uniform(0,1) < self.cicles_time/(1+10000*max(self.body.knows.tell_nests()[1], self.body.knows.tell_nests()[3])*(len(self.space.nearby(i,self.r,self.rng,type(self.body)))+1)):
                    self.GoTo.nw = "wander"
                    self.body.knows.set_state('GoToNest')
                    if self.body.knows.tell_nests()[0] == self.current_quadrant:
                        self.current_quadrant = self.body.knows.tell_nests()[2]
                    else:
                        self.current_quadrant = self.body.knows.tell_nests()[0]
                    self.cicles_time = 0
            self.body.fc = 'm'
            i=self.space.bodindex(self.body.name)
            neigh=self.space.nearby(i,self.r,self.rng,type(self.body))
            for n in neigh:
                nests = n.knows.tell_nests()
                self.body.knows.add_nest(nests[0], nests[1])
                self.body.knows.add_nest(nests[2], nests[3])
            if len(self.space.incontact(i,Nest))>0:
                self.GoTo.nw = "zigzag"
                self.cicles_count = self.cicles_count + 1
                self.body.knows.add_nest(self.getQuadrant(), self.cicles_count)
            else:
                self.GoTo.nw = "wander"
                self.body.knows.add_nest(self.getQuadrant(), self.cicles_count)
                self.cicles_count = 0
                nest=self.space.nearest(i,self.r,self.rng,Nest)
                if nest!=None:
                    self.GoTo.cmd_set(self.space.nearestpoint(i,nest)) 
        elif self.body.knows.tell_state()=='OtherQuadrant':
            self.body.fc = 'c'
            self.cicles_time = 0
            neigh=self.space.nearby(i,self.r,self.rng,type(self.body))
            for n in neigh:
                nests = n.knows.tell_nests()
                self.body.knows.add_nest(nests[0], nests[1])
                self.body.knows.add_nest(nests[2], nests[3])
            if self.current_quadrant == self.getQuadrant():
                self.cicles_time = 0
                self.body.knows.set_state(self.after_reaching_quadrant)
                self.GoTo.nw = "wander"
        if self.current_quadrant != self.getQuadrant() and self.body.knows.tell_state()!='OtherQuadrant':
            self.after_reaching_quadrant = self.body.knows.tell_state()
            self.body.knows.set_state('OtherQuadrant')
            self.GoTo.cmd_set(self.getPointQuadrant())

                
        # if self.body.knows.tell_state()=='ThereAreNests': # changes color and starts moving faster and more straight
        #     self.body.fc='m' # this is NOT the usual way to show a soul, see Voronoid or GoTo for examples
        #     self.body.set_vel_max(self.body.v_max*2,self.body.w_max/4)
        #     self.body.cmd_vel(self.body.v_max,0)
        #     self.GoTo.cmd_set(None)
        self.cicles_time = self.cicles_time + 1
        super().update()

## MAIN

name='Ex1'+strftime("%Y%m%d%H%M", localtime())
s=Space(name,T=RT,limits='')
p=KPIdata(name,2,TS)
state_kpi = []
# a big nest in the second quadrant:
bignest=Nest('BigNest',pos=(uniform(-0.8*W,-0.2*W),uniform(0.4*H,0.6*H)),area=6) # 
smallnest=Nest('SmallNest',pos=(uniform(0.8*W,0.2*W),uniform(-0.4*H,-0.6*H)),area=2) 
s.bodies.append(bignest)
s.bodies.append(smallnest)
N=100
s.spawn_bodies(nm=N)
for b in s.bodies:
    if isinstance(b,Mobot):
        b.cmd_vel(v=b.v_max/2)
        Demo(b,s,TS/10,r=2,rng=np.pi) # a new Soul for b (who assigns new Knowledge too)
KPI=[0,1] # [ fraction of live mobots who know ThereAreNests , fraction of initial robots still alive ]
STATES = [0,0,0,0,0,0]
end=False
while not end:
    for b in s.bodies:
        if isinstance(b,Mobot):
            b.update()
    s.update_dist()
    # s.remobodies(s.collisions())
    s.update_conn()
    if visual and time()>s.t0+(s.updates+1)*s.T:
        s.redraw()
    if time()>p.t0+(p.updates+1)*p.T:
        KPI=[0,0]
        STATES = [0,0,0,0,0,0]
        for b in s.bodies:
            if isinstance(b,Mobot):
                KPI[1] += 1
                if b.knows.tell_state()=='RestInNest' and b.pos.x < 0 and b.pos.y > 0:
                    KPI[0] += 1
                if b.knows.tell_state() == 'KnowNothing':
                    STATES[0] += 1
                elif b.knows.tell_state() == 'KnowOneNest':
                    STATES[1] += 1
                elif b.knows.tell_state() == 'KnowTwoNest':
                    STATES[2] += 1
                elif b.knows.tell_state() == 'GoToNest':
                    STATES[3] += 1
                elif b.knows.tell_state() == 'RestInNest':
                    STATES[4] += 1
                elif b.knows.tell_state() == 'OtherQuadrant':
                    STATES[5] += 1
        KPI[0] /= KPI[1]
        KPI[1] /= N
        p.update(KPI)
        STATES = np.array(STATES)/N
        state_kpi.append(STATES)
    end=s.has_been_closed() or KPI[0]>0.9 or KPI[1]<0.1 
    np.save("states.npy", np.array(state_kpi))
else:
    s.close()
    p.close()
