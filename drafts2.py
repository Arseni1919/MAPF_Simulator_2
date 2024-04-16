from globals import *

list1 = [1,2,3]
list2 = [2,3,4]
print(len(set(list1).intersection(list2)))
print(list(set(list1).intersection(list2)))