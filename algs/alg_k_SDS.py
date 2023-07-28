import logging
import random
import time
import matplotlib.pyplot as plt
import cProfile
import pstats

import numpy as np

from algs.alg_a_star import a_star
from algs.test_mapf_alg import test_mapf_alg_from_pic
from algs.metrics import c_v_check_for_agent, c_e_check_for_agent, build_constraints, \
    limit_is_crossed, get_agents_in_conf, check_plan, get_alg_info_dict, iteration_print


class KSDSAgent:
    def __init__(self, index, start_node, goal_node, nodes, nodes_dict, h_func,
                 plotter, middle_plot,
                 iter_limit=1e100, map_dim=None):
        self.index = index
        self.name = f'agent_{index}'
        self.start_node = start_node
        self.goal_node = goal_node
        self.nodes = nodes
        self.nodes_dict = nodes_dict
        self.h_func = h_func
        self.plotter = plotter
        self.middle_plot = middle_plot
        self.iter_limit = iter_limit
        self.path = []
        self.other_paths = {}
        self.conf_paths = {}
        self.map_dim = map_dim
        self.stats_n_closed = 0
        self.stats_n_calls = 0
        self.stats_runtime = 0
        self.stats_n_messages = 0
        self.stats_confs_per_iter = []

    def exchange(self, agents):
        self.other_paths = {agent.name: agent.path for agent in agents if agent.name != self.name}
        self.stats_n_messages += len(agents)

    def decision_bool(self, agents_in_confs, agents_dict, decision_type='max_prev', **kwargs):
        alpha = kwargs['alpha'] if 'alpha' in kwargs else 0.5

        if len(self.path) == 0:
            return True

        if len(agents_in_confs) == 0:
            raise RuntimeError('len(agents_in_confs) == 0')

        if decision_type == 'simple':
            if random.random() < alpha:
                return True

        elif decision_type == 'max_prev':
            # A MORE SMART VERSION
            path_lngths = [len(self.other_paths[agent_name]) for agent_name in agents_in_confs]
            max_n = max(path_lngths)
            min_n = min(path_lngths)
            # priority on smaller paths
            if len(self.path) > max_n and random.random() < 0.1:
                return True
            elif len(self.path) < min_n and random.random() < 0.9:
                return True
            else:
                path_lngths.append(len(self.path))
                path_lngths.sort()
                my_order = path_lngths.index(len(self.path))
                my_alpha = 0.9 - 0.8 * (my_order / len(path_lngths))
                if random.random() < my_alpha:
                    return True
            return False

        else:
            raise RuntimeError('no such decision_type')

        return False

    def plan(self, agents_dict=None, decision_type='max_prev', **kwargs):
        start_time = time.time()

        c_v_list = c_v_check_for_agent(self.name, self.path, self.other_paths)
        c_e_list = c_e_check_for_agent(self.name, self.path, self.other_paths)
        self.stats_confs_per_iter.append(len(c_v_list) + len(c_e_list))

        if len(self.path) > 0 and len(c_v_list) == 0 and len(c_e_list) == 0:
            # print(f'\n ---------- (DS {decision_type}) NO NEED FOR A* {self.name} ---------- \n')
            return False, {'elapsed': None, 'a_s_info': None}, True

        agents_in_confs = get_agents_in_conf(c_v_list, c_e_list)
        to_change = self.decision_bool(agents_in_confs, agents_dict, decision_type, **kwargs)
        if to_change:
            v_constr_dict, e_constr_dict, perm_constr_dict = build_constraints(self.nodes, self.other_paths)
            print(f'\n ---------- ({kwargs["alg_name"]}) A* {self.name} ---------- \n')
            a_star_func = kwargs['a_star_func']
            new_path, a_s_info = a_star_func(start=self.start_node, goal=self.goal_node,
                                             nodes=self.nodes, nodes_dict=self.nodes_dict, h_func=self.h_func,
                                             v_constr_dict=v_constr_dict,
                                             e_constr_dict=e_constr_dict,
                                             perm_constr_dict=perm_constr_dict,
                                             plotter=self.plotter, middle_plot=self.middle_plot,
                                             iter_limit=self.iter_limit)
            # stats
            self.stats_n_calls += 1
            self.stats_n_closed += a_s_info['n_closed']
            self.stats_runtime += time.time() - start_time
            if new_path is not None:
                self.path = new_path
            return True, {'elapsed': time.time() - start_time, 'a_s_info': a_s_info,
                          'n_agents_conf': len(agents_in_confs)}, False

        # print(f'\n ---------- (DS {decision_type}) NO NEED FOR A* {self.name} ---------- \n')
        return False, {'elapsed': None, 'a_s_info': None}, False


def create_agents(start_nodes, goal_nodes, nodes, nodes_dict, h_func, plotter, middle_plot, iter_limit, map_dim):
    # Creating agents
    agents = []
    agents_dict = {}
    n_agent = 0
    for start_node, goal_node in zip(start_nodes, goal_nodes):
        agent = KSDSAgent(n_agent, start_node, goal_node, nodes, nodes_dict, h_func, plotter, middle_plot, iter_limit,
                          map_dim)
        agents.append(agent)
        agents_dict[agent.name] = agent
        n_agent += 1

    return agents, agents_dict


def check_if_limit_is_crossed(func_info, alg_info, **kwargs):
    runtime = 0
    # TODO: update alg_info with func_info
    return limit_is_crossed(runtime, alg_info, **kwargs)


def run_k_sds(start_nodes, goal_nodes, nodes, nodes_dict, h_func, **kwargs):
    runtime = 0
    iter_limit = kwargs['a_star_iter_limit'] if 'a_star_iter_limit' in kwargs else 1e100
    plotter = kwargs['plotter'] if 'plotter' in kwargs else None
    middle_plot = kwargs['middle_plot'] if 'middle_plot' in kwargs else False
    final_plot = kwargs['final_plot'] if 'final_plot' in kwargs else True
    plot_per = kwargs['plot_per'] if 'plot_per' in kwargs else 10
    map_dim = kwargs['map_dim'] if 'map_dim' in kwargs else None
    alg_name = kwargs['alg_name'] if 'alg_name' in kwargs else f'k-SDS'
    k_step_iteration_limit = kwargs['k_step_iteration_limit'] if 'k_step_iteration_limit' in kwargs else 1000

    # Creating agents
    agents, agents_dict = create_agents(start_nodes, goal_nodes, nodes, nodes_dict, h_func, plotter, middle_plot, iter_limit, map_dim)

    # alg info dict
    alg_info = get_alg_info_dict()

    # Distributed Part
    for k_step_iteration in range(1000000):

        func_info = all_plan_and_find_nei()  # agents
        if check_if_limit_is_crossed(func_info, alg_info, **kwargs):
            return None, {'agents': agents, 'success_rate': 0}

        there_are_collisions = True
        while there_are_collisions:

            there_are_collisions, func_info = all_exchange_k_step_paths()  # agents
            if check_if_limit_is_crossed(func_info, alg_info, **kwargs):
                return None, {'agents': agents, 'success_rate': 0}

            if not there_are_collisions:
                break
            func_info = all_replan()  # agents
            if check_if_limit_is_crossed(func_info, alg_info, **kwargs):
                return None, {'agents': agents, 'success_rate': 0}

        all_paths_are_finished, func_info = all_move_k_steps()  # agents
        if check_if_limit_is_crossed(func_info, alg_info, **kwargs):
            return None, {'agents': agents, 'success_rate': 0}

        plans = {agent.name: agent.path for agent in agents}
        iteration_print(agents, plans, alg_name, alg_info, runtime, k_step_iteration)
        if all_paths_are_finished:
            there_is_col, c_v, c_e, cost = check_plan(agents, plans, alg_name, alg_info, runtime, k_step_iteration)
            if not there_is_col:
                if final_plot:
                    print(f'#########################################################')
                    print(f'#########################################################')
                    print(f'#########################################################')
                    plotter.plot_mapf_paths(paths_dict=plans, nodes=nodes, plot_per=plot_per)
                alg_info['success_rate'] = 1
                alg_info['sol_quality'] = cost
                alg_info['runtime'] = runtime
                alg_info['a_star_calls_per_agent'] = [agent.stats_n_calls for agent in agents]
                alg_info['n_messages_per_agent'] = [agent.stats_n_messages for agent in agents]
                alg_info['confs_per_iter'] = np.sum([agent.stats_confs_per_iter for agent in agents], 0).tolist()
            return plans, alg_info
        if k_step_iteration > k_step_iteration_limit:
            return None, {'agents': agents, 'success_rate': 0}
        return None, {'agents': agents, 'success_rate': 0}


def main():
    random_seed = True
    # random_seed = False
    seed = 277
    n_agents = 50
    PLOT_PER = 10

    to_use_profiler = True
    # to_use_profiler = False

    # DECISION_TYPE = 'simple'
    DECISION_TYPE = 'max_prev'

    # img_dir = 'empty-48-48.map'  # 48-48
    img_dir = 'random-64-64-10.map'  # 64-64
    # img_dir = 'warehouse-10-20-10-2-1.map'  # 63-161
    # img_dir = 'lt_gallowstemplar_n.map'  # 180-251

    profiler = cProfile.Profile()
    if to_use_profiler:
        profiler.enable()
    for i in range(3):
        print(f'\n[run {i}]')
        result, info = test_mapf_alg_from_pic(
            algorithm=run_k_sds,
            img_dir=img_dir,
            n_agents=n_agents,
            random_seed=random_seed,
            seed=seed,
            final_plot=True,
            a_star_iter_limit=5e7,
            max_time=5,
            plot_per=PLOT_PER,
            alg_name='k-SDS',
            limit_type='norm_time',
            a_star_func=a_star,
            decision_type=DECISION_TYPE
        )

        if not random_seed:
            break

        # plt.show()
        plt.close()

    if to_use_profiler:
        profiler.disable()
        stats = pstats.Stats(profiler).sort_stats('cumtime')
        stats.dump_stats('../stats/results_k_sds.pstat')


if __name__ == '__main__':
    main()
