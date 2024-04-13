from functions import *
import cProfile
import pstats
from typing import *
from algs.test_mapf_alg import test_mapf_alg_from_pic
from algs.metrics import c_v_check_for_agent, c_e_check_for_agent, build_constraints, \
    limit_is_crossed, get_agents_in_conf, check_plan, get_alg_info_dict, iteration_print
from algs.metrics import just_check_k_step_plans, just_check_plans


class BnBAgent:
    def __init__(self, index, start_node, goal_node, nodes, nodes_dict, h_func):
        self.index = index
        self.start_node = start_node
        self.curr_node = start_node
        self.goal_node = goal_node
        self.nodes = nodes
        self.nodes_dict = nodes_dict
        self.h_func = h_func
        self.path = []

        # stats
        self.stats_n_closed = 0
        self.stats_n_calls = 0
        self.stats_runtime = 0
        self.stats_n_messages = 0
        self.stats_n_step_m = 0
        self.stats_n_step_m_list = []
        self.stats_nei_list = []
        # nei
        self.nei_list = []
        self.nei_dict = {}
        self.nei_paths_dict = {}

    @property
    def name(self):
        return f'agent_{self.index}'

    @property
    def path_names(self):
        return [n.xy_name for n in self.path]


def run_bnb(start_nodes, goal_nodes, nodes, nodes_dict, h_func, **kwargs) -> Tuple[Dict[str, list] | None, dict]:
    """
    :param start_nodes:
    :param goal_nodes:
    :param nodes:
    :param nodes_dict:
    :param h_func:
    :param kwargs:
    - plotter
    - middle_plot
    - map_dim
    - final_plot
    - max_time
    - a_star_iter_limit
    - a_star_calls_limit
    - a_star_closed_nodes_limit
    - alg_name
    - i_run
    - img_dir
    :return:
    """
    max_time = kwargs['max_time']  # seconds

    # preps
    alg_info = get_alg_info_dict()
    start_time = time.time()

    # create agents
    agents = []
    for a_index, (s_node, g_node) in enumerate(zip(start_nodes, goal_nodes)):
        agent = BnBAgent(a_index, s_node, g_node, nodes, nodes_dict, h_func)
        agents.append(agent)

    # step iterations
    step = 0
    while True:
        step += 1

        # Build a pseudo-tree
        pass

        # find the best future move
        pass

        # execute the move + check (+ plot)
        pass

        # check if everybody arrived -> return
        pass

        # time check -> return
        runtime = time.time() - start_time
        if runtime > max_time:
            return None, {'agents': agents, 'success_rate': 0}

        # print + plot
        print(f'\r{step=}, {runtime=:.2f}', end='')

    # return None, {'agents': agents, 'success_rate': 0}


def main():
    n_agents = 200
    # img_dir = 'my_map_10_10_room.map'  # 10-10
    # img_dir = 'empty-48-48.map'  # 48-48
    img_dir = 'random-32-32-10.map'  # 32-32
    # img_dir = 'random-64-64-10.map'  # 64-64
    # img_dir = 'warehouse-10-20-10-2-1.map'  # 63-161
    # img_dir = 'lt_gallowstemplar_n.map'  # 180-251
    # img_dir = 'random-32-32-10.map'  # 32-32               | LNS |
    # img_dir = 'ht_chantry.map'  # 162-141   | Up to 230 agents with h=w=30, lim=10sec.

    # random_seed = True
    random_seed = False
    seed = 878
    PLOT_PER = 1
    PLOT_RATE = 0.5

    # --------------------------------------------------- #
    # --------------------------------------------------- #
    # for the algorithms
    alg_name = f'B&B'
    # --------------------------------------------------- #
    # --------------------------------------------------- #

    to_use_profiler = True
    # to_use_profiler = False
    profiler = cProfile.Profile()
    if to_use_profiler:
        profiler.enable()
    for i in range(3):
        print(f'\n[run {i}]')
        result, info = test_mapf_alg_from_pic(
            algorithm=run_bnb,
            img_dir=img_dir,
            alg_name=alg_name,
            n_agents=n_agents,
            random_seed=random_seed,
            seed=seed,
            final_plot=True,
            a_star_iter_limit=5e7,
            # limit_type='norm_time',
            limit_type='dist_time',
            max_time=50,
            a_star_closed_nodes_limit=1e6,
            plot_per=PLOT_PER,
            plot_rate=PLOT_RATE,
        )

        if not random_seed:
            break

        # plt.show()
        plt.close()

    if to_use_profiler:
        profiler.disable()
        stats = pstats.Stats(profiler).sort_stats('cumtime')
        stats.dump_stats('../stats/results_bnb.pstat')


if __name__ == '__main__':
    main()

