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


def get_list_sol_q_style(statistics_dict, alg_name, n_agents, list_type, runs_per_n_agents, algs_to_test_list,
                         is_json=False):
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
            to_insert = True
            for another_alg in algs_to_test_list:
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


def get_list_a_star(statistics_dict, alg_name, n_agents_list, list_type, is_json=False):
    """
    {
        alg_name: {
            n_agents: {
                'success_rate': {run: None for run in range(runs_per_n_agents)},
                'sol_quality': {run: None for run in range(runs_per_n_agents)},
                'runtime': {run: None for run in range(runs_per_n_agents)},
                'iterations_time': {run: None for run in range(runs_per_n_agents)},
                'a_star_calls_counter': {run: None for run in range(runs_per_n_agents)},
                'a_star_calls_dist_counter': {run: None for run in range(runs_per_n_agents)},
                'a_star_runtimes': [],
                'a_star_n_closed': [],
            } for n_agents in n_agents_list
        } for alg_name, _ in algs_to_test_dict.items()
    }
    """
    curr_list = []
    for n_agents in n_agents_list:
        if is_json:
            n_agents = str(n_agents)
        curr_element = statistics_dict[alg_name][n_agents][list_type]
        if curr_element is not None:
            curr_list.extend(curr_element)
    return curr_list


class Plotter:
    def __init__(self, map_dim=None, subplot_rows=2, subplot_cols=4):
        if map_dim:
            self.side_x, self.side_y = map_dim
        self.subplot_rows = subplot_rows
        self.subplot_cols = subplot_cols
        self.fig, self.ax = plt.subplots(subplot_rows, subplot_cols, figsize=(14, 7))

    def plot_lists(self, open_list, closed_list, start, goal=None, path=None, nodes=None, a_star_run=False):

        field = np.zeros((self.side_x, self.side_y))
        # for ax in self.ax:
        #     ax.cla()
        self.cla_axes()

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
            if a_star_run:
                field[node.x, node.y] = node.g
            else:
                field[node.x, node.y] = node.g_dict[start.xy_name]
        self.ax[1].imshow(field, origin='lower')
        self.ax[1].set_title('open_list')

        field = np.zeros((self.side_x, self.side_y))
        for node in closed_list:
            if a_star_run:
                field[node.x, node.y] = node.g
            else:
                field[node.x, node.y] = node.g_dict[start.xy_name]
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
            if t % plot_per == 0:
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
                    t_path = path[:t + 1]
                    for node in t_path:
                        field[node.x, node.y] = 3
                    self.ax.scatter(t_path[-1].y, t_path[-1].x, s=100, c='k')
                    self.ax.scatter(t_path[-1].y, t_path[-1].x, s=50, c=np.array([color_map(i)]))
                    i += 1

                for agent_name, path in paths_dict.items():
                    field[path[0].x, path[0].y] = 4
                    field[path[-1].x, path[-1].y] = 5

                self.ax.imshow(field, origin='lower')
                self.ax.set_title('MAPF Paths')
                # plt.pause(1)
                plt.pause(0.01)

    def cla_axes(self):
        if self.ax.ndim == 1:
            for i_ax in self.ax:
                i_ax.cla()
        if self.ax.ndim == 2:
            for row_ax in self.ax:
                for col_ax in row_ax:
                    col_ax.cla()

    @staticmethod
    def plot_success_rate(ax, statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, is_json=False):
        for alg_name, (alg_func, alg_info) in algs_to_test_dict.items():
            # success_rate
            sr_x = []
            sr_y = []
            for n_agents in n_agents_list:
                sr_list = get_list_n_run(statistics_dict, alg_name, n_agents, 'success_rate', runs_per_n_agents,
                                         is_json)
                if len(sr_list) > 0:
                    sr_x.append(n_agents)
                    sr_y.append(sum(sr_list) / len(sr_list))
            ax.plot(sr_x, sr_y, '-o', label=f'{alg_name}', alpha=0.75)

        ax.set_title('success_rate')
        ax.set_xlim([min(n_agents_list) - 1, max(n_agents_list) + 1])
        # ax.yaxis.set_major_locator(MaxNLocator(integer=True))
        # ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        ax.set_ylim([0, 1.5])
        ax.set_xticks(n_agents_list)
        ax.set_xlabel('N agents')
        ax.legend()

    @staticmethod
    def plot_sol_quality(ax, statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, is_json=False):
        for alg_name, (alg_func, alg_info) in algs_to_test_dict.items():

            # sol_quality
            sq_x = []
            sq_y = []
            for n_agents in n_agents_list:
                sq_list = get_list_sol_q_style(statistics_dict, alg_name, n_agents, 'sol_quality', runs_per_n_agents,
                                               list(algs_to_test_dict.keys()), is_json)
                if len(sq_list) > 0:
                    sq_x.append(n_agents)
                    sq_y.append(np.mean(sq_list))
            ax.plot(sq_x, sq_y, '-o', label=f'{alg_name}', alpha=0.75)

        ax.set_title('sol_quality')
        ax.set_xlim([min(n_agents_list) - 1, max(n_agents_list) + 1])
        ax.set_xticks(n_agents_list)
        ax.set_xlabel('N agents')
        ax.legend()

    @staticmethod
    def plot_runtime(ax, statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, is_json=False):
        max_instances = 0
        for alg_name, (alg_func, alg_info) in algs_to_test_dict.items():

            # runtime
            rt_y = get_list_runtime(statistics_dict, alg_name, n_agents_list, 'runtime', runs_per_n_agents, is_json)
            rt_y.sort()
            rt_x = list(range(len(rt_y)))
            max_instances = max(max_instances, len(rt_x))
            ax.plot(rt_x, rt_y, '-o', label=f'{alg_name}', alpha=0.75)
            if len(rt_x) > 0:
                ax.text(rt_x[-1], rt_y[-1], f'{rt_x[-1] + 1}', bbox=dict(facecolor='yellow', alpha=0.75))

            # dist_runtime
            if alg_info['dist']:
                it_y = get_list_runtime(statistics_dict, alg_name, n_agents_list, 'dist_runtime', runs_per_n_agents,
                                        is_json)
                it_y.sort()
                it_x = list(range(len(it_y)))
                max_instances = max(max_instances, len(it_x))
                ax.plot(it_x, it_y, '--^', label=f'{alg_name} (dist)')
                if len(it_x) > 0:
                    ax.text(it_x[-1], it_y[-1], f'{it_x[-1] + 1}', bbox=dict(facecolor='yellow', alpha=0.75))

        ax.set_title('runtime (cactus)')
        ax.set_xlim([0, max_instances + 2])
        # ax.set_xticks(rt_x)
        ax.set_xlabel('Solved Instances')
        ax.set_yscale('log')
        ax.legend()

    @staticmethod
    def plot_a_star_calls_counters(ax, statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list,
                                   is_json=False):
        max_instances = 0
        for alg_name, (alg_func, alg_info) in algs_to_test_dict.items():

            # A* calls
            ac_y = get_list_runtime(statistics_dict, alg_name, n_agents_list, 'a_star_calls_counter', runs_per_n_agents,
                                    is_json)
            ac_y.sort()
            ac_x = list(range(len(ac_y)))
            max_instances = max(max_instances, len(ac_x))
            ax.plot(ac_x, ac_y, '-o', label=f'{alg_name}', alpha=0.75)
            if len(ac_x) > 0:
                ax.text(ac_x[-1], ac_y[-1], f'{ac_x[-1] + 1}',
                        bbox=dict(facecolor='yellow', alpha=0.75))

            if alg_info['dist']:
                # get_list_a_star(statistics_dict, alg_name, n_agents_list, list_type, is_json=False)
                acd_y = get_list_a_star(statistics_dict, alg_name, n_agents_list, 'a_star_calls_counter_dist', is_json)
                acd_y.sort()
                acd_x = list(range(len(acd_y)))
                max_instances = max(max_instances, len(acd_x))
                ax.plot(acd_x, acd_y, '--^', label=f'{alg_name} (dist)')
                if len(acd_x) > 0:
                    ax.text(acd_x[-1], acd_y[-1], f'{acd_x[-1] + 1}', bbox=dict(facecolor='yellow', alpha=0.75))

        ax.set_title('A* Calls (cactus)')
        ax.set_xlim([0, max_instances + 2])
        ax.set_xlabel('Solved Instances')
        ax.set_yscale('log')
        ax.legend()

    @staticmethod
    def plot_a_star_calls_boxplot(ax, statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list,
                                  is_json=False):
        # showfliers = False
        showfliers = True
        big_table = []
        for alg_name, (alg_func, alg_info) in algs_to_test_dict.items():
            a_star_calls = []
            for n_agents in n_agents_list:
                if is_json:
                    n_agents = str(n_agents)
                a_star_calls.extend(statistics_dict[alg_name][n_agents]['a_star_calls_per_agent'])
            big_table.append(a_star_calls)
        ax.boxplot(big_table,
                   # notch=True,  # notch shape
                   vert=True,  # vertical box alignment
                   patch_artist=True,  # fill with color
                   # labels=algs_to_test_list,  # linewidth=0.5, edgecolor="white"
                   showfliers=showfliers)

        # ax.set_title('a_star_runtimes (boxplot)')
        # ax.set_xlim([0, max_instances + 2])
        # ax.set_xticks()
        ax.set_xticklabels(algs_to_test_dict.keys(), rotation=15)
        ax.set_ylabel(f'A * calls average per agent')
        ax.set_xlabel(f'{"with" if showfliers else "no"} outliers')
        # ax.legend()

    @staticmethod
    def plot_n_closed_cactus(ax, statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, is_json=False):
        max_instances = 0
        for alg_name, (alg_func, alg_info) in algs_to_test_dict.items():

            # runtime
            rt_y = get_list_a_star(statistics_dict, alg_name, n_agents_list, 'n_closed_per_run', is_json)
            rt_y.sort()
            rt_x = list(range(len(rt_y)))
            max_instances = max(max_instances, len(rt_x))
            ax.plot(rt_x, rt_y, '-o', label=f'{alg_name}', alpha=0.75)
            if len(rt_x) > 0:
                ax.text(rt_x[-1], rt_y[-1], f'{rt_x[-1] + 1}', bbox=dict(facecolor='yellow', alpha=0.75))

            if alg_info['dist']:
                l_y = get_list_a_star(statistics_dict, alg_name, n_agents_list, 'a_star_n_closed_dist', is_json)
                l_y.sort()
                l_x = list(range(len(l_y)))
                max_instances = max(max_instances, len(l_x))
                ax.plot(l_x, l_y, '--^', label=f'{alg_name} (dist)')
                if len(l_x) > 0:
                    ax.text(l_x[-1], l_y[-1], f'{l_x[-1] + 1}', bbox=dict(facecolor='yellow', alpha=0.75))

        # ax.set_title('n_closed (cactus)')
        ax.set_xlim([0, max_instances + 2])
        # ax.set_xticks(rt_x)
        ax.set_ylabel('n_closed')
        ax.set_xlabel('Solved Instances')
        ax.set_yscale('log')
        ax.legend()

    @staticmethod
    def plot_n_messages_boxplot(ax, statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, is_json):
        # showfliers = False
        showfliers = True
        big_table = []
        algs_names = []
        for alg_name, (alg_func, alg_info) in algs_to_test_dict.items():
            if alg_info['dist']:
                algs_names.append(alg_name)
                n_messages = []
                for n_agents in n_agents_list:
                    if is_json:
                        n_agents = str(n_agents)
                    n_messages.extend(statistics_dict[alg_name][n_agents]['n_messages_per_agent'])
                big_table.append(n_messages)
        ax.boxplot(big_table,
                   # notch=True,  # notch shape
                   vert=True,  # vertical box alignment
                   patch_artist=True,  # fill with color
                   # labels=algs_to_test_list,  # linewidth=0.5, edgecolor="white"
                   showfliers=showfliers)

        # ax.set_title('a_star_runtimes (boxplot)')
        # ax.set_xlim([0, max_instances + 2])
        # ax.set_xticks()
        ax.set_xticklabels(algs_names, rotation=15)
        ax.set_ylabel(f'N messages average per agent')
        ax.set_xlabel(f'{"with" if showfliers else "no"} outliers')
        # ax.legend()

    @staticmethod
    def plot_n_agents_conf(ax, statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, is_json):
        for alg_name, (alg_func, alg_info) in algs_to_test_dict.items():
            l_x = []
            l_y = []
            for n_agents in n_agents_list:
                if is_json:
                    n_agents = str(n_agents)
                if 'n_agents_conf' in statistics_dict[alg_name][n_agents]:
                    n_agents_conf = statistics_dict[alg_name][n_agents]['n_agents_conf']
                    if len(n_agents_conf) > 0:
                        l_y.append(np.mean(statistics_dict[alg_name][n_agents]['n_agents_conf']))
                        l_x.append(n_agents)

            if len(l_y) > 0:
                ax.plot(l_x, l_y, '-o', label=f'{alg_name}', alpha=0.75)

        ax.set_ylabel('n_agents_conf')
        ax.set_xlim([min(n_agents_list) - 1, max(n_agents_list) + 1])
        ax.set_xticks(n_agents_list)
        ax.set_xlabel('N agents')
        ax.legend()

    @staticmethod
    def plot_conf_per_iter(ax, statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, is_json, **kwargs):
        min_x, max_x = 0, 0
        for alg_name, (alg_func, alg_info) in algs_to_test_dict.items():
            if alg_info['dist'] and 'n_agents' in kwargs:
                l_y = statistics_dict[alg_name][kwargs['n_agents']]['confs_per_iter']
                l_x = list(range(len(l_y)))
                min_x = min(min_x, min(l_x))
                max_x = max(max_x, max(l_x))

                if len(l_y) > 0:
                    ax.plot(l_x, l_y, '-o', label=f'{alg_name}', alpha=0.75)

        ax.set_ylabel('Conflicts per Iteration')
        ax.set_xlim([min_x - 1, max_x + 1])
        ax.set_xticks(n_agents_list)
        ax.set_xlabel('Iterations')
        ax.legend()

    def plot_big_test(self, statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, img_png='',
                      is_json=False, **kwargs):
        print('big plot starts')
        self.cla_axes()
        self.plot_success_rate(
            self.ax[0, 0], statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, is_json)

        self.plot_sol_quality(
            self.ax[0, 1], statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, is_json)

        self.plot_runtime(
            self.ax[0, 2], statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, is_json)

        self.plot_a_star_calls_counters(
            self.ax[0, 3], statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, is_json)

        self.plot_a_star_calls_boxplot(
            self.ax[1, 0], statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, is_json)

        self.plot_n_messages_boxplot(
            self.ax[1, 1], statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, is_json)

        self.plot_n_closed_cactus(
            self.ax[1, 2], statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, is_json)

        # self.plot_n_agents_conf(
        #     self.ax[1, 3], statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, is_json)

        self.plot_conf_per_iter(
            self.ax[1, 3], statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, is_json, **kwargs)

        # self.fig.tight_layout()
        self.fig.suptitle(f'{img_png} Map, {runs_per_n_agents} RpP', fontsize=16)
        plt.pause(0.001)
        print('big plot ends')
        # plt.show()
