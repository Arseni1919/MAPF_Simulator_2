import matplotlib.pyplot as plt
import numpy as np


class Plotter:
    def __init__(self, map_dim):
        self.side_x, self.side_y = map_dim
        self.fig, self.ax = plt.subplots(1, 3)

    def plot_lists(self, open_list, closed_list, start, goal=None, path=None, nodes=None):

        field = np.zeros((self.side_x, self.side_y))
        for ax in self.ax:
            ax.cla()

        if nodes:
            for node in nodes:
                field[node.x, node.y] = -1

        for node in open_list:
            field[node.x, node.y] = 1

        for node in closed_list:
            field[node.x, node.y] = 2

        if path:
            for node in path:
                field[node.x, node.y] = 3

        field[start.x, start.y] = 4
        if goal:
            field[goal.x, goal.y] = 5

        self.ax[0].imshow(field, origin='lower')
        self.ax[0].set_title('general')

        # if path:
        #     for node in path:
        #         field[node.x, node.y] = 3
        #         self.ax[0].text(node.x, node.y, f'{node.ID}', bbox={'facecolor': 'yellow', 'alpha': 1, 'pad': 10})

        field = np.zeros((self.side_x, self.side_y))
        for node in open_list:
            field[node.x, node.y] = node.g
        self.ax[1].imshow(field, origin='lower')
        self.ax[1].set_title('open_list')

        field = np.zeros((self.side_x, self.side_y))
        for node in closed_list:
            field[node.x, node.y] = node.g
        self.ax[2].imshow(field, origin='lower')
        self.ax[2].set_title('closed_list')

        self.fig.tight_layout()
        # plt.pause(1)
        plt.pause(0.01)
        # plt.show()





