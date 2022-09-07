import matplotlib.pyplot as plt

n = 10
l = plt.cm.get_cmap('hsv', n)
for i in range(n):
    item = l(i)
    print(item)