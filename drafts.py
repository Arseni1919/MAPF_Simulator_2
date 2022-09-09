# import matplotlib.pyplot as plt
#
# n = 10
# l = plt.cm.get_cmap('hsv', n)
# for i in range(n):
#     item = l(i)
#     print(item)
import random

l = [0, 1, 2, 3]
l.insert(3, 100)
random.shuffle(l)
print(l)
print(l.index(100))