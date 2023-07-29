# from globals import *

# class Node:
#     def __init__(self):
#         self.num = 123
# node = Node()
# l = [node, node, node, node]
k = 7
path = [1]
while len(path) < k:
    path.append(path[-1] + 1)
print(path)
print(path[:k])
print(path[k-1:])