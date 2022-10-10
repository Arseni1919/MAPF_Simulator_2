import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator


def get_list_n_run(statistics_dict, alg_name, n_agents, list_type, runs_per_n_agents, is_json=False):
    """
    {
        alg_name: {
            n_agents: {
                'success_rate': {run: None for run in range(runs_per_n_agents)},
                'sol_quality': {run: None for run in range(runs_per_n_agents)},
                'runtime': {run: None for run in range(runs_per_n_agents)},
            } for n_agents in n_agents_list
        } for alg_name, _ in algs_to_test_dict.items()
    }
    """
    curr_list = []
    for i_run in range(runs_per_n_agents):
        if is_json:
            n_agents = str(n_agents)
            i_run = str(i_run)
        curr_element = statistics_dict[alg_name][n_agents][list_type][i_run]
        if curr_element is not None:
            curr_list.append(curr_element)
    return curr_list


def get_list_sol_q_style(statistics_dict, alg_name, n_agents, list_type, runs_per_n_agents, algs_to_test_dict, is_json=False):
    """
    {
        alg_name: {
            n_agents: {
                'success_rate': {run: None for run in range(runs_per_n_agents)},
                'sol_quality': {run: None for run in range(runs_per_n_agents)},
                'runtime': {run: None for run in range(runs_per_n_agents)},
            } for n_agents in n_agents_list
        } for alg_name, _ in algs_to_test_dict.items()
    }
    """
    algs_list = list(algs_to_test_dict.keys())
    curr_list = []
    for i_run in range(runs_per_n_agents):
        if is_json:
            n_agents = str(n_agents)
            i_run = str(i_run)
        curr_element = statistics_dict[alg_name][n_agents][list_type][i_run]
        if curr_element is not None:
            to_insert = True
            for another_alg in algs_list:
                another_element = statistics_dict[another_alg][n_agents][list_type][i_run]
                if another_element is None:
                    to_insert = False
                    break
            if to_insert:
                curr_list.append(curr_element)
    return curr_list


def get_list_runtime(statistics_dict, alg_name, n_agents_list, list_type, runs_per_n_agents, is_json=False):
    """
    {
        alg_name: {
            n_agents: {
                'success_rate': {run: None for run in range(runs_per_n_agents)},
                'sol_quality': {run: None for run in range(runs_per_n_agents)},
                'runtime': {run: None for run in range(runs_per_n_agents)},
            } for n_agents in n_agents_list
        } for alg_name, _ in algs_to_test_dict.items()
    }
    """
    curr_list = []
    for n_agents in n_agents_list:
        for i_run in range(runs_per_n_agents):
            if is_json:
                n_agents = str(n_agents)
                i_run = str(i_run)
            curr_element = statistics_dict[alg_name][n_agents][list_type][i_run]
            if curr_element is not None:
                curr_list.append(curr_element)
    return curr_list


class Plotter:
    def __init__(self, map_dim=None, subplot_rows=1, subplot_cols=4):
        if map_dim:
            self.side_x, self.side_y = map_dim
        self.subplot_rows = subplot_rows
        self.subplot_cols = subplot_cols
        self.fig, self.ax = plt.subplots(subplot_rows, subplot_cols, figsize=(14, 7))

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

    def plot_mapf_paths(self, paths_dict, nodes=None, plot_per=10):
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

            n = len(list(paths_dict.keys()))
            color_map = plt.cm.get_cmap('hsv', n)
            i = 0
            for agent_name, path in paths_dict.items():
                t_path = path[:t+1]
                for node in t_path:
                    field[node.x, node.y] = 3
                self.ax.scatter(t_path[-1].y, t_path[-1].x, s=100, c='k')
                self.ax.scatter(t_path[-1].y, t_path[-1].x, s=50, c=np.array([color_map(i)]))
                i += 1

            for agent_name, path in paths_dict.items():
                field[path[0].x, path[0].y] = 4
                field[path[-1].x, path[-1].y] = 5

            if t % plot_per == 0:
                self.ax.imshow(field, origin='lower')
                self.ax.set_title('MAPF Paths')

                # plt.pause(1)
                plt.pause(0.01)

    def plot_big_test(self, statistics_dict, runs_per_n_agents, algs_to_test_list, n_agents_list, img_png='', is_json=False):
        for i_ax in self.ax:
            i_ax.cla()
        # self.fig, self.ax = plt.subplots(self.subplot_rows, self.subplot_cols)
        max_instances = 0
        for alg_name in algs_to_test_list:

            # success_rate
            sr_x = []
            sr_y = []
            for n_agents in n_agents_list:
                sr_list = get_list_n_run(statistics_dict, alg_name, n_agents, 'success_rate', runs_per_n_agents, is_json)
                if len(sr_list) > 0:
                    sr_x.append(n_agents)
                    sr_y.append(sum(sr_list) / len(sr_list))
            self.ax[0].plot(sr_x, sr_y, '-o', label=f'{alg_name}')

            # sol_quality
            sq_x = []
            sq_y = []
            for n_agents in n_agents_list:
                sq_list = get_list_sol_q_style(statistics_dict, alg_name, n_agents, 'sol_quality', runs_per_n_agents, algs_to_test_dict, is_json)
                if len(sq_list) > 0:
                    sq_x.append(n_agents)
                    sq_y.append(np.mean(sq_list))
            self.ax[1].plot(sq_x, sq_y, '-o', label=f'{alg_name}')

            # runtime
            rt_y = get_list_runtime(statistics_dict, alg_name, n_agents_list, 'runtime', runs_per_n_agents, is_json)
            rt_y.sort()
            rt_x = list(range(len(rt_y)))
            max_instances = max(max_instances, len(rt_x))
            self.ax[2].plot(rt_x, rt_y, '-o', label=f'{alg_name}')
            if len(rt_x) > 0:
                self.ax[2].text(rt_x[-1], rt_y[-1], f'{rt_x[-1] + 1}', bbox=dict(facecolor='yellow', alpha=0.75))

            # iterations_time
            if 'DS-MAPF' in alg_name:
                it_y = get_list_runtime(statistics_dict, alg_name, n_agents_list, 'iterations_time', runs_per_n_agents, is_json)
                it_y.sort()
                it_x = list(range(len(it_y)))
                max_instances = max(max_instances, len(it_x))
                self.ax[2].plot(it_x, it_y, '-o', label=f'{alg_name} (iteration time)')
                if len(it_x) > 0:
                    self.ax[2].text(it_x[-1], it_y[-1], f'{it_x[-1] + 1}', bbox=dict(facecolor='yellow', alpha=0.75))

            # A* calls
            ac_y = get_list_runtime(statistics_dict, alg_name, n_agents_list, 'a_star_calls_counter', runs_per_n_agents,
                                    is_json)
            ac_y.sort()
            ac_x = list(range(len(ac_y)))
            max_instances = max(max_instances, len(ac_x))
            self.ax[3].plot(ac_x, ac_y, '-o', label=f'{alg_name}')
            if len(ac_x) > 0:
                self.ax[3].text(ac_x[-1], ac_y[-1], f'{ac_x[-1] + 1}',
                                bbox=dict(facecolor='yellow', alpha=0.75))

            if 'DS-MAPF' in alg_name:
                acd_y = get_list_runtime(statistics_dict, alg_name, n_agents_list, 'a_star_calls_dist_counter', runs_per_n_agents, is_json)
                acd_y.sort()
                acd_x = list(range(len(acd_y)))
                max_instances = max(max_instances, len(acd_x))
                self.ax[3].plot(acd_x, acd_y, '-o', label=f'{alg_name} (distributed)')
                if len(acd_x) > 0:
                    self.ax[3].text(acd_x[-1], acd_y[-1], f'{acd_x[-1] + 1}', bbox=dict(facecolor='yellow', alpha=0.75))

        self.ax[0].set_title('success_rate')
        self.ax[1].set_title('sol_quality')
        self.ax[2].set_title('runtime (cactus)')
        self.ax[3].set_title('A* Calls (cactus)')

        self.ax[0].set_xlim([min(n_agents_list) - 1, max(n_agents_list) + 1])
        self.ax[0].set_ylim([0, 1.5])
        self.ax[1].set_xlim([min(n_agents_list) - 1, max(n_agents_list) + 1])
        self.ax[2].set_xlim([0, max_instances + 2])
        self.ax[3].set_xlim([0, max_instances + 2])

        self.ax[0].set_xticks(n_agents_list)
        # self.ax[0].yaxis.set_major_locator(MaxNLocator(integer=True))
        # self.ax[0].xaxis.set_major_locator(MaxNLocator(integer=True))
        self.ax[1].set_xticks(n_agents_list)
        # self.ax[2].set_xticks(rt_x)

        self.ax[0].set_xlabel('N agents')
        self.ax[1].set_xlabel('N agents')
        self.ax[2].set_xlabel('Solved Instances')
        self.ax[3].set_xlabel('Solved Instances')

        self.ax[0].legend()
        self.ax[1].legend()
        self.ax[2].legend()
        self.ax[3].legend()

        # self.fig.tight_layout()
        self.fig.suptitle(f'{img_png} Map', fontsize=16)
        plt.pause(0.001)
        # plt.show()







