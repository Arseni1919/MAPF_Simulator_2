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

    def f(self):
        return self.t + self.h
        # return self.g + self.h


