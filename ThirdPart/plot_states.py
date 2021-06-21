import numpy as np
import matplotlib.pyplot as plt

plt.style.use('classic')


name = "exp6_r4"
states = np.load("output/" + name + "/states.npy")
print(np.shape(states))
print(np.max(states))

fig, ax = plt.subplots()
ax.plot(states[:,0], '-k', label='KnowNothing')
ax.plot(states[:,1], '-r', label='KnowOneNest')
ax.plot(states[:,2], '-g', label='KnowTwoNest')
ax.plot(states[:,3], '-b', label='GoToNest')
ax.plot(states[:,4], '-m', label='RestInNest')
ax.plot(states[:,5], '-c', label='OtherQuadrant')
leg = ax.legend()
# plt.show()
plt.savefig("output/" + name + "/states.png")