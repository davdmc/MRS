import numpy as np
import matplotlib.pyplot as plt

plt.style.use('classic')


name = "time"
metric1 = np.load("1ag-1ta/" + name + ".npy")
metric2 = np.load("2-2/" + name + ".npy")
metric3 = np.load("4-4/" + name + ".npy")
metric4 = np.load("10-10/" + name + ".npy")
fig, ax = plt.subplots()
ax.plot([1, 2, 4, 10], [np.mean(metric1), np.mean(metric2), np.mean(metric3), np.mean(metric4)], 'o-b', label='Execution time of the algorithm')
leg = ax.legend()
# plt.show()
plt.savefig(name + ".png")



# name = "cost_node"
# metric = np.load("2-4/" + name + ".npy")
# fig, ax = plt.subplots()
# ax.set_ylim([0, 0.00000000000005])
# ax.plot(metric, '-b', label='Minimum cost of node in the tree')
# leg = ax.legend()
# # plt.show()
# plt.savefig("2-4/" + name + ".png")

# name = "det_covariance"
# metric = np.load("2-4/" + name + ".npy")
# fig, ax = plt.subplots()
# ax.plot(metric, '-b', label='Det covariance of the state per step')
# leg = ax.legend()
# # plt.show()
# plt.savefig("2-4/" + name + ".png")

# name = "error_pos"
# metric = np.load("2-4/" + name + ".npy")
# fig, ax = plt.subplots()
# ax.plot(metric, '-b', label='Error in state estimation per step')
# leg = ax.legend()
# # plt.show()
# plt.savefig("2-4/" + name + ".png")

# name = "length_path_chosen"
# metric = np.load("2-4/" + name + ".npy")
# fig, ax = plt.subplots()
# ax.plot(metric, '-b', label='Length of the path chosen')
# leg = ax.legend()
# # plt.show()
# plt.savefig("2-4/" + name + ".png")