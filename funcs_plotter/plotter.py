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

    def plot_mapf_paths(self, paths_dict, nodes=None):
        plt.close()
        self.fig, self.ax = plt.subplots()
        longest_path = max([len(path) for path in paths_dict.values()])

        for t in range(longest_path):
            field = np.zeros((self.side_x, self.side_y))
            self.ax.cla()
            # for ax in self.ax:
            #     ax.cla()

            if nodes:
                for node in nodes:
                    field[node.x, node.y] = -1

            for agent_name, path in paths_dict.items():
                t_path = path[:t+1]
                for node in t_path:
                    field[node.x, node.y] = 3
                self.ax.scatter(t_path[-1].y, t_path[-1].x, s=100, c='red')

            for agent_name, path in paths_dict.items():
                field[path[0].x, path[0].y] = 4
                field[path[-1].x, path[-1].y] = 5

            self.ax.imshow(field, origin='lower')
            self.ax.set_title('MAPF Paths')

            # plt.pause(1)
            plt.pause(0.01)
            # plt.show()






