#!/usrbin/env python3

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import psutil
import collections

cpu = collections.deque(np.zeros(10))
ram = collections.deque(np.zeros(10))
print("CPU: {}".format(cpu))
print("Memory: {}".format(ram))

# function to update the data
def my_function():
    cpu.popleft()
    cpu.append(psutil.cpu_percent(interval=1))
    ax.plot(cpu)
    ram.popleft()
    ram.append(psutil.virtual_memory().percent)
    ax1.plot(ram)# start collections with zeros

cpu = collections.deque(np.zeros(10))
ram = collections.deque(np.zeros(10))# define and adjust figure
fig = plt.figure(figsize=(12,6), facecolor='#DEDEDE')
ax = plt.subplot(121)
ax1 = plt.subplot(122)
ax.set_facecolor('#DEDEDE')
ax1.set_facecolor('#DEDEDE')# test
my_function()
plt.show()