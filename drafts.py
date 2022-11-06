# import heapq
#
# l = []
#
# class Node:
#     def __init__(self):
#         self.ID = 'a'
#
# heapq.heappush(l, ((1, 1), Node()))
# heapq.heappush(l, ((1, 1), Node()))
# heapq.heappush(l, ((1, 3), Node()))
# heapq.heappush(l, ((1, 4), Node()))
# # heapq.heappush(l, (2, 'C'))
# # heapq.heappush(l, (3, 'D'))
# print(f'heap_list: {l}')
# popped = heapq.heappop(l)
# print(f'popped: {popped}')
# popped = heapq.heappop(l)
# print(f'popped: {popped}')
import random
print(bool(random.getrandbits(1)))