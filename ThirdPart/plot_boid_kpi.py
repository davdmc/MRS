import numpy as np
import matplotlib.pyplot as plt
from matplotlib import style
from numpy.core.function_base import linspace
import os

style.use('ggplot')

# Show plots?
show = True

# Introduce exp directory
directory = './working_dir/kpi1/'
experiments = []
num_exp = 0

# Obtain all the experiments files
for filename in os.listdir(directory):
    if filename.endswith(".csv"):
        num_exp += 1
        experiments.append(np.loadtxt(os.path.join(directory,filename), delimiter=','))
    else:
        continue

# Compute avg of experiments
avg_KPIs = np.zeros(experiments[0].shape)
for experiment in experiments:
    avg_KPIs += experiment
avg_KPIs /= num_exp

dev_KPIs = np.zeros(experiments[0].shape)
for experiment in experiments:
    dev_KPIs += (experiment - avg_KPIs) ** 2

dev_KPIs /= num_exp
dev_KPIs = np.sqrt(dev_KPIs)

total_time = 5*60
step = 0.1
number = int(total_time/step)

x = linspace(0, total_time, number)

# Fraction of alive / per biggest connected KPI[0,1]
plt.ylim([0, 1.2])
plt.title("Alive")
plt.ylabel("Fraction alive")
plt.xlabel("Time [s]")
plt.legend(["Per N", "In largest connected"])
plt.errorbar(x, avg_KPIs[1:,0], dev_KPIs[1:,0], errorevery=30)
plt.errorbar(x, avg_KPIs[1:,1], dev_KPIs[1:,1], errorevery=30)
plt.savefig("results/alive.png", dpi=400)
if show:
    plt.show()

# Cohesion radius KPI[2]
plt.title("Cohesion radius")
plt.ylabel("Radius [m]")
plt.xlabel("Time [s]")
plt.legend(["Cohesion radius"])
plt.errorbar(x, avg_KPIs[1:,2], dev_KPIs[1:,2], errorevery=30)
plt.savefig("results/radius.png", dpi=400)
if show:
    plt.show()

# Distance error
plt.title("Distance error")
plt.ylabel("MSE Error")
plt.xlabel("Time [s]")
plt.legend(["Distance error"])
plt.errorbar(x, avg_KPIs[1:,3], dev_KPIs[1:,3], errorevery=30)
plt.savefig("results/results.png", dpi=400)
if show:
    plt.show()

# Influence steering
plt.title("Steering components influence")
plt.ylabel("Influence")
plt.xlabel("Time [s]")
plt.errorbar(x, avg_KPIs[1:,4], dev_KPIs[1:,4], errorevery=30)
plt.errorbar(x, avg_KPIs[1:,5], dev_KPIs[1:,5], errorevery=30)
plt.errorbar(x, avg_KPIs[1:,6], dev_KPIs[1:,6], errorevery=30)
plt.errorbar(x, avg_KPIs[1:,7], dev_KPIs[1:,7], errorevery=30)
plt.legend(["Cohesion", "Repulsion", "Bearing", "Direction"])
plt.savefig("results/steering.png", dpi=400)
if show:
    plt.show()
