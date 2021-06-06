## (Not so) Basic Voronoids
## Version: 20210524
## Author: Enrique Teruel (ET) eteruel@unizar.es
## License: CC-BY-SA

from time import time_ns, time, localtime, strftime
import numpy as np
from random import uniform
from matplotlib.animation import FFMpegWriter # requires having ffmpeg installed, from https://ffmpeg.org/

from small_worl2d_config import visual, TS, RT, W, H, room
from small_worl2d import Space, KPIdata, Obstacle, Mobot, Voronoid

## MAIN

name='BasicVoronoids'+strftime("%Y%m%d%H%M", localtime())
s=Space(name,T=RT,R=0,limits='') # No comm
p=KPIdata(name,1,2*TS)
new=Obstacle('O',pos=-1,area=-1,vertices=[(-W/4,-H),(W/4,-H),(W/4,H),(-W/4,H)],fc='k')
s.bodies.append(new)
s.spawn_bodies(100) # Mobots only
s.remobodies([0])
for b in s.bodies:
    if isinstance(b,Mobot):
        Voronoid(b,s,r=4,T=uniform(TS,2*TS),Q=room) # rng=uniform(0.75*np.pi,0.95*np.pi) would be WithoutRearView, hardly "works"
KPI=[1]
end=False
while not end:
    for b in s.bodies:
        if isinstance(b,Mobot):
            b.update()
    s.update_dist()
    s.remobodies(s.collisions())
    s.update_conn()
    if visual and time()>s.t0+(s.updates+1)*s.T:
        s.redraw()
    if time()>p.t0+(p.updates+1)*p.T:
        KPI=[0] # max distance moved
        for b in s.bodies:
            if isinstance(b,Mobot):
                KPI[0] = max(KPI[0],b.souls[1].d)
        p.update(KPI)
    end=s.has_been_closed() or p.updates>50
else:
    s.close()
    p.close()
