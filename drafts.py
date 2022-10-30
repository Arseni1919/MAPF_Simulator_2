import matplotlib.pyplot as plt
import numpy as np

# plt.style.use('_mpl-gallery')

# make data
np.random.seed(1)
x_1 = 4 + np.random.normal(0, 1.5, 2000)
x_2 = 10 + np.random.normal(0, 1.5, 2000)

# plot:
fig, ax = plt.subplots()

ax.hist(x_1, bins=20, linewidth=0.5, edgecolor="white", label='bla', alpha=0.5)
ax.hist(x_2, bins=20, linewidth=0.5, edgecolor="white", label='blu', alpha=0.5)

# ax.set(xlim=(0, 8), xticks=np.arange(1, 8), ylim=(0, 56), yticks=np.linspace(0, 56, 9))

ax.legend()
plt.show()