import logging
import random
import time
import matplotlib.pyplot as plt
import cProfile
import pstats

import numpy as np

from algs.alg_a_star import a_star
from algs.test_mapf_alg import test_mapf_alg_from_pic
from algs.metrics import check_for_collisions, c_v_check_for_agent, c_e_check_for_agent, build_constraints, \
    crossed_time_limit, get_agents_in_conf


class DSAgent:
    def __init__(self, index, start_node, goal_node, nodes, nodes_dict, h_func,
                 plotter, middle_plot,
                 iter_limit=1e100, map_dim=None, limit_type='simple'):
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
        self.limit_type = limit_type

    def exchange(self, agents):
        self.other_paths = {agent.name: agent.path for agent in agents if agent.name != self.name}

    def decision_bool(self, alpha, decision_type, agents_in_confs, new_path=None):

        if len(self.path) == 0:
            # self.path = new_path
            return True

        if len(agents_in_confs) == 0:
            raise RuntimeError('len(agents_in_confs) == 0')

        if decision_type == 'simple':
            if random.random() < alpha:
                # self.path = new_path
                return True

        if decision_type == 'opt_1':
            path_lngths = [len(self.other_paths[agent_name]) for agent_name in agents_in_confs]
            max_n = max(path_lngths)
            min_n = min(path_lngths)
            # priority on smaller paths
            if len(self.path) > max_n and random.random() < 0.9:
                # self.path = new_path
                return True
            elif len(self.path) < min_n and random.random() < 0.1:
                # self.path = new_path
                return True
            elif random.random() < alpha:
                # self.path = new_path
                return True
            return

        if decision_type == 'opt_2':
            path_lngths = [len(self.other_paths[agent_name]) for agent_name in agents_in_confs]
            max_n = max(path_lngths)
            min_n = min(path_lngths)
            # priority on bigger paths
            if len(self.path) > max_n and random.random() < 0.1:
                # self.path = new_path
                return True
            elif len(self.path) < min_n and random.random() < 0.9:
                # self.path = new_path
                return True
            elif random.random() < alpha:
                # self.path = new_path
                return True
        return False

    def get_a_star_iter_limit(self, agents_in_confs):
        iter_limit = self.iter_limit
        if self.limit_type == 'simple':
            return iter_limit
        if self.limit_type == 'smart':
            if len(agents_in_confs) > 0 and self.map_dim:
                agent_in_conf_paths_len = [len(self.other_paths[agent]) for agent in agents_in_confs]
                max_path_len = max(agent_in_conf_paths_len)
                alt_iter_limit = max_path_len * self.map_dim[0] * self.map_dim[1]
                iter_limit = min(iter_limit, alt_iter_limit)
        return iter_limit

    def plan(self, alpha, decision_type):
        start_time = time.time()

        c_v_list = c_v_check_for_agent(self.name, self.path, self.other_paths)
        c_e_list = c_e_check_for_agent(self.name, self.path, self.other_paths)

        if len(self.path) > 0 and len(c_v_list) == 0 and len(c_e_list) == 0:
            print(f'\n ---------- (DS) NO NEED FOR A* {self.name} ---------- \n')
            return False, {'elapsed': None, 'a_s_info': None}

        agents_in_confs = get_agents_in_conf(c_v_list, c_e_list)
        to_change = self.decision_bool(alpha, decision_type, agents_in_confs)
        if to_change:
            v_constr_dict, e_constr_dict, perm_constr_dict = build_constraints(self.nodes, self.other_paths)
            iter_limit = self.get_a_star_iter_limit(agents_in_confs)
            print(f'\n ---------- (DS) A* {self.name}, iter limit: {iter_limit} ---------- \n')
            new_path, a_s_info = a_star(start=self.start_node, goal=self.goal_node,
                                        nodes=self.nodes, nodes_dict=self.nodes_dict, h_func=self.h_func,
                                        v_constr_dict=v_constr_dict,
                                        e_constr_dict=e_constr_dict,
                                        perm_constr_dict=perm_constr_dict,
                                        plotter=self.plotter, middle_plot=self.middle_plot,
                                        iter_limit=iter_limit)
            if new_path is not None:
                self.path = new_path
            return True, {'elapsed': time.time() - start_time, 'a_s_info': a_s_info, 'n_agents_conf': len(agents_in_confs) }

        print(f'\n ---------- NO NEED FOR A* {self.name} ---------- \n')
        return False, {'elapsed': None, 'a_s_info': None}


def run_ds_mapf(start_nodes, goal_nodes, nodes, nodes_dict, h_func, plotter=None, middle_plot=False, **kwargs):
    start_time = time.time()
    iterations_time = 0
    if 'max_time' in kwargs:
        max_time = kwargs['max_time']
    else:
        max_time = 60
    if 'final_plot' in kwargs:
        final_plot = kwargs['final_plot']
    else:
        final_plot = True
    if 'plot_per' in kwargs:
        plot_per = kwargs['plot_per']
    else:
        plot_per = 10
    if 'a_star_iter_limit' in kwargs:
        iter_limit = kwargs['a_star_iter_limit']
    else:
        iter_limit = 1e100
    if 'a_star_calls_limit' in kwargs:
        a_star_calls_limit = kwargs['a_star_calls_limit']
    else:
        a_star_calls_limit = 1e100
    if 'map_dim' in kwargs:
        map_dim = kwargs['map_dim']
    else:
        map_dim = None
    if 'limit_type' in kwargs:
        limit_type = kwargs['limit_type']
    else:
        limit_type = 'simple'
    a_star_calls_counter = 0
    a_star_calls_dist_counter = 0
    a_star_runtimes = []
    a_star_n_closed = []
    n_agents_conf_list = []
    if 'alpha' in kwargs:
        alpha = kwargs['alpha']
    else:
        alpha = 0.5
    if 'decision_type' in kwargs:
        decision_type = kwargs['decision_type']
    else:
        decision_type = 'opt_1'
    # Creating agents
    agents = []
    n_agent = 0
    for start_node, goal_node in zip(start_nodes, goal_nodes):
        agent = DSAgent(n_agent, start_node, goal_node, nodes, nodes_dict, h_func, plotter, middle_plot, iter_limit, map_dim, limit_type)
        agents.append(agent)
        n_agent += 1

    # Distributed Part
    for iteration in range(1000000):
        max_time_list = []
        # time constraint
        # if crossed_time_limit(start_time, max_time * len(agents)):
        if crossed_time_limit(start_time, max_time):
            break

        if a_star_calls_counter >= a_star_calls_limit:
            break
        # if a_star_calls_dist_counter >= a_star_calls_limit:
        #     break

        # PLAN
        for agent in agents:
            succeeded, info = agent.plan(alpha=alpha, decision_type=decision_type)
            if info['elapsed']:
                max_time_list.append(info['elapsed'])
                a_star_runtimes.append(info['a_s_info']['runtime'])
                a_star_n_closed.append(info['a_s_info']['n_closed'])
                if iteration > 0:
                    n_agents_conf_list.append(info['n_agents_conf'])
                a_star_calls_counter += 1
        if len(max_time_list) > 0:
            iterations_time += max(max_time_list)
            a_star_calls_dist_counter += 1

        # EXCHANGE
        for agent in agents:
            agent.exchange(agents=agents)

        # CHECK PLAN
        plan = {agent.name: agent.path for agent in agents}
        plan_lngths = [len(path) for path in plan.values()]
        if 0 in plan_lngths:
            raise RuntimeError('0 in plan_lngths')
        cost = sum([len(path) for path in plan.values()])
        print(f'\r---\n'
              f'[DS-MAPF ({decision_type})][{len(agents)} agents][A* calls: {a_star_calls_counter}][A* dist calls: {a_star_calls_dist_counter}][time: {time.time() - start_time:0.2f}s][iter {iteration}]\n'
              f'cost: {cost}\n'
              f'---\n')
        if cost:
            there_is_col, c_v, c_e = check_for_collisions(plan)
            if not there_is_col:
                if final_plot:
                    print(f'#########################################################')
                    print(f'#########################################################')
                    print(f'#########################################################')
                    plotter.plot_mapf_paths(paths_dict=plan, nodes=nodes, plot_per=plot_per)
                return plan, {'agents': agents,
                              'success_rate': 1,
                              'sol_quality': cost,
                              'runtime': (time.time() - start_time),
                              'iterations_time': iterations_time,
                              'a_star_calls_counter': a_star_calls_counter,
                              'a_star_calls_dist_counter': a_star_calls_dist_counter,
                              'a_star_runtimes': a_star_runtimes,
                              'a_star_n_closed': a_star_n_closed,
                              'n_agents_conf': n_agents_conf_list}

    # partial order
    pass

    return None, {'agents': agents, 'success_rate': 0}


def main():
    profiler = cProfile.Profile()
    if to_use_profiler:
        profiler.enable()
    for i in range(3):
        print(f'\n[run {i}]')
        result, info = test_mapf_alg_from_pic(algorithm=run_ds_mapf, initial_ordering=[], n_agents=n_agents,
                                              random_seed=random_seed, seed=seed, final_plot=True,
                                              a_star_iter_limit=5e7, max_time=5, plot_per=PLOT_PER, limit_type='smart')

        if not random_seed:
            break

        # plt.show()
        plt.close()

    if to_use_profiler:
        profiler.disable()
        stats = pstats.Stats(profiler).sort_stats('cumtime')
        stats.dump_stats('../stats/results_ds_mapf.pstat')


if __name__ == '__main__':
    random_seed = True
    # random_seed = False
    seed = 277
    n_agents = 150
    PLOT_PER = 1
    to_use_profiler = True
    # to_use_profiler = False

    # good example: img_dir = '19_20_warehouse.png'
    # random_seed = True
    # random_seed = False
    # seed = 23
    # n_agents = 30

    main()

# sub_results = {k: v for k, v in self.other_paths.items() if len(v) > 0}
# conf_agents = []
# conf_list = []
# c_v_list = c_v_check_for_agent(self.name, self.path, sub_results)
# c_e_list = c_e_check_for_agent(self.name, self.path, sub_results)
# conf_list.extend(c_v_list)
# conf_list.extend(c_e_list)
# for conf in conf_list:
#     if self.name != conf[0]:
#         conf_agents.append(conf[0])
#     if self.name != conf[1]:
#         conf_agents.append(conf[1])
#
# self.conf_paths = {agent_name: self.other_paths[agent_name] for agent_name in conf_agents}
# if len(self.conf_paths) > 0:
#     print()


# sub_results = {}
# no_paths_agents = []
# for k, v in self.other_paths.items():
#     if len(v) > 0:
#         sub_results[k] = v
#     else:
#         no_paths_agents.append(k)

# c_v_list_after_1 = c_v_check_for_agent(self.name, new_path, self.conf_paths)
# c_e_list_after_1 = c_e_check_for_agent(self.name, new_path, self.conf_paths)

# c_v_list_after_2 = c_v_check_for_agent(self.name, new_path, self.other_paths)
# c_e_list_after_2 = c_e_check_for_agent(self.name, new_path, self.other_paths)
# if len(c_v_list_after_2) > 0 or len(c_e_list_after_2) > 0:
#     raise RuntimeError('a_star failed')

# no_paths_agents = [k for k, v in self.other_paths.items() if len(v) == 0]
# if len(no_paths_agents) > 0:
#     raise RuntimeError('len(no_paths_agents) > 0')


