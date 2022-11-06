import random

class Node:
    def __init__(self, x, y, t=0, neighbours=None, new_ID=None):
        if new_ID:
            self.ID = new_ID
        else:
            self.ID = f'{x}_{y}_{t}'
        self.xy_name = f'{x}_{y}'
        self.x = x
        self.y = y
        self.t = t
        if neighbours is None:
            self.neighbours = []
        else:
            self.neighbours = neighbours
        # self.neighbours = neighbours

        self.h = 0
        self.g = t
        self.parent = None
        self.g_dict = {}

    def f(self):
        return self.t + self.h
        # return self.g + self.h

    def reset(self, target_nodes=None):
        self.t = 0
        self.h = 0
        self.g = self.t
        self.ID = f'{self.x}_{self.y}_{self.t}'
        self.parent = None
        if target_nodes is not None:
            self.g_dict = {target_node.xy_name: 0 for target_node in target_nodes}
        else:
            self.g_dict = {}

    # def decision_rule(self, other):
    #     # return bool(random.getrandbits(1))
    #     return self.x > other.x or self.y > other.y
    #
    # def __lt__(self, other):
    #     return self.decision_rule(other)
    #
    # def __le__(self, other):
    #     return self.decision_rule(other)
    #
    # def __eq__(self, other):
    #     return self.decision_rule(other)
    #
    # def __ne__(self, other):
    #     return self.decision_rule(other)
    #
    # def __gt__(self, other):
    #     return self.decision_rule(other)
    #
    # def __ge__(self, other):
    #     return self.decision_rule(other)




