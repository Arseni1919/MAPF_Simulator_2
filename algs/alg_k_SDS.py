import random
from typing import List

import cProfile
import pstats


from functions import *

from algs.alg_space_time_a_star import a_star
from algs.test_mapf_alg import test_mapf_alg_from_pic
from algs.metrics import c_v_check_for_agent, c_e_check_for_agent, build_constraints, \
    limit_is_crossed, get_agents_in_conf, check_plan, just_check_plans, get_alg_info_dict, iteration_print


class KSDSAgent:
    def __init__(self, index, start_node, goal_node, nodes, nodes_dict, h_func,
                 plotter, middle_plot,
                 iter_limit=1e100, map_dim=None):
        self.index = index
        self.name = f'agent_{index}'
        self.start_node = start_node
        self.curr_node = start_node
        self.goal_node = goal_node
        self.nodes = nodes
        self.nodes_dict = nodes_dict
        self.h_func = h_func
        self.plotter = plotter
        self.middle_plot = middle_plot
        self.iter_limit = iter_limit
        self.path = []
        self.full_path = []
        self.map_dim = map_dim

        self.stats_n_closed = 0
        self.stats_n_calls = 0
        self.stats_runtime = 0
        self.stats_n_messages = 0
        self.stats_confs_per_iter = []
        # nei
        self.nei_list = []
        self.nei_dict = {}
        self.nei_paths_dict = {}
        self.conf_agents_names = []

    def update_nei(self, agents, **kwargs):
        k = kwargs['k']
        self.conf_agents_names = []
        self.nei_list, self.nei_dict, self.nei_paths_dict = [], {}, {}
        nei_dist_const = 2 * k + 1
        for agent in agents:
            if agent.name != self.name:
                curr_distance = manhattan_distance_nodes(self.curr_node, agent.curr_node)
                if curr_distance <= nei_dist_const:
                    self.nei_list.append(agent)
                    self.nei_dict[agent.name] = agent
                    self.nei_paths_dict[agent.name] = None

    def calc_a_star_plan(self, v_constr_dict=None, e_constr_dict=None, perm_constr_dict=None, **kwargs):
        start_time = time.time()
        if not v_constr_dict:
            v_constr_dict = {node.xy_name: [] for node in self.nodes}
        if not e_constr_dict:
            e_constr_dict = {node.xy_name: [] for node in self.nodes}
        if not perm_constr_dict:
            perm_constr_dict = {node.xy_name: [] for node in self.nodes}

        print(f'\n ---------- ({kwargs["alg_name"]}) [k_step_iter: {kwargs["k_step_iteration"]}][small_iter: {kwargs["small_iteration"]}] A* {self.name} ---------- \n')
        a_star_func = kwargs['a_star_func']
        new_path, a_s_info = a_star_func(start=self.curr_node, goal=self.goal_node,
                                         nodes=self.nodes, nodes_dict=self.nodes_dict, h_func=self.h_func,
                                         v_constr_dict=v_constr_dict,
                                         e_constr_dict=e_constr_dict,
                                         perm_constr_dict=perm_constr_dict,
                                         plotter=self.plotter, middle_plot=self.middle_plot,
                                         iter_limit=self.iter_limit)
        if new_path is not None:
            self.path = new_path
            succeeded = True
        else:
            # self.path = [self.curr_node]
            succeeded = False
        return succeeded, {'a_s_time': time.time() - start_time, 'a_s_info': a_s_info}

    def exchange_paths(self):
        for nei in self.nei_list:
            # nei.nei_paths_dict[agent.name] = agent.path[:k]
            nei.nei_paths_dict[self.name] = self.path

    def update_conf_agents_names(self, **kwargs):
        k = kwargs['k']
        self.conf_agents_names = []
        init_nei_k_steps_paths_dict = {agent_name: path[:k] for agent_name, path in self.nei_paths_dict.items()}
        conf_list = c_v_check_for_agent(self.name, self.path[:k], init_nei_k_steps_paths_dict)
        c_e_list = c_e_check_for_agent(self.name, self.path[:k], init_nei_k_steps_paths_dict)
        conf_list.extend(c_e_list)
        conf_agents_names = []
        for conf in conf_list:
            conf_agents_names.append(conf[1])
        self.conf_agents_names = list(set(conf_agents_names))

    def get_nei_k_steps_paths_dict(self, **kwargs):
        k = kwargs['k']
        nei_k_steps_paths_dict = {}
        # use self.conf_agents_names and kwargs['small_iteration']
        for agent_name, path in self.nei_paths_dict.items():
            if agent_name in self.conf_agents_names:
                nei_k_steps_paths_dict[agent_name] = path[:k + k * round(kwargs['small_iteration'] / 10)]
                continue
            nei_k_steps_paths_dict[agent_name] = path[:k]
        return nei_k_steps_paths_dict

    def get_paths_to_consider_dict(self, **kwargs):
        p_h, p_l = 0.9, 0.1
        paths_to_consider_dict = {}
        nei_k_steps_paths_dict = self.get_nei_k_steps_paths_dict(**kwargs)
        for agent_name, path in self.nei_paths_dict.items():
            if len(self.path) > len(path):
                if random.random() < p_l:
                    paths_to_consider_dict[agent_name] = nei_k_steps_paths_dict[agent_name]
            elif len(self.path) < len(path):
                if random.random() < p_h:
                    paths_to_consider_dict[agent_name] = nei_k_steps_paths_dict[agent_name]
            elif self.index > self.nei_dict[agent_name].index:
                if random.random() < p_l:
                    paths_to_consider_dict[agent_name] = nei_k_steps_paths_dict[agent_name]
            else:
                if random.random() < p_h:
                    paths_to_consider_dict[agent_name] = nei_k_steps_paths_dict[agent_name]
        return paths_to_consider_dict

    def replan(self, **kwargs):
        self.update_conf_agents_names(**kwargs)
        if len(self.conf_agents_names) == 0:
            return False, {}
        # probabilities to use: p_ch, p_h, p_l
        p_ch = self.set_p_ch(**kwargs)
        if random.random() < p_ch:
            paths_to_consider_dict = self.get_paths_to_consider_dict(**kwargs)
            v_constr_dict, e_constr_dict, perm_constr_dict = build_constraints(self.nodes, paths_to_consider_dict)
            succeeded, info = self.calc_a_star_plan(v_constr_dict, e_constr_dict, perm_constr_dict, **kwargs)
            return succeeded, info
        return False, {}

    def set_p_h_and_p_l(self, **kwargs):
        p_h, p_l = 0.9, 0.1
        h_names, l_names = [], []
        for agent_name, path in self.nei_paths_dict.items():
            if len(self.path) > len(path):
                l_names.append(agent_name)
            elif len(self.path) < len(path):
                h_names.append(agent_name)
            elif self.index > self.nei_dict[agent_name].index:
                l_names.append(agent_name)
            else:
                h_names.append(agent_name)
        return p_h, p_l, h_names, l_names

    def set_p_ch(self, decision_type='max_prev', **kwargs):
        alpha = kwargs['alpha'] if 'alpha' in kwargs else 0.5

        if len(self.path) == 0:
            return 1

        if decision_type == 'simple':
            return alpha

        elif decision_type == 'max_prev':
            # A MORE SMART VERSION
            path_lngths = []
            for agent_name, path in self.nei_paths_dict.items():
                path_lngths.append(len(path))
            path_lngths.append(len(self.path))
            path_lngths.sort()
            my_order = path_lngths.index(len(self.path))
            my_alpha = 0.9 - 0.8 * (my_order / (len(path_lngths) - 1))
            return my_alpha

        else:
            raise RuntimeError('no such decision_type')

    def update_full_path(self, **kwargs):
        k = kwargs['k']
        goal_name = self.goal_node.xy_name

        while len(self.path) < k:
            self.path.append(self.path[-1])

        self.full_path.extend(self.path[:k])
        self.curr_node = self.full_path[-1]
        self.path = None
        return self.curr_node.xy_name == goal_name

    def cut_back_full_path(self):
        # if self.name == 'agent_2':
        #     print()
        len_full_path = len(self.full_path)
        if len_full_path > 1:
            if self.full_path[-1].xy_name == self.goal_node.xy_name:
                for backwards_node in self.full_path[:-1][::-1]:
                    if backwards_node.xy_name == self.goal_node.xy_name:
                        len_full_path -= 1
                    else:
                        break
        self.full_path = self.full_path[:len_full_path]


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

    # runtime - the sequential time in seconds - number
    if 'runtime' in func_info:
        alg_info['runtime'] += func_info['runtime']
    # alg_info['dist_runtime'] - distributed time in seconds - number
    if 'dist_runtime' in func_info:
        alg_info['dist_runtime'] += func_info['dist_runtime']

    # alg_info['a_star_calls_counter'] - number
    if 'a_star_calls_counter' in func_info:
        alg_info['a_star_calls_counter'] += func_info['a_star_calls_counter']
    # alg_info['a_star_calls_counter_dist'] - number
    if 'a_star_calls_counter_dist' in func_info:
        alg_info['a_star_calls_counter_dist'] += func_info['a_star_calls_counter_dist']

    # alg_info['a_star_n_closed'] - list
    if 'a_star_n_closed' in func_info:
        alg_info['a_star_n_closed'].append(func_info['a_star_n_closed'])
    # alg_info['a_star_n_closed_dist'] - number
    if 'a_star_n_closed_dist' in func_info:
        alg_info['a_star_n_closed_dist'] += func_info['a_star_n_closed_dist']

    return limit_is_crossed(alg_info['runtime'], alg_info, **kwargs)


def all_plan_and_find_nei(agents: List[KSDSAgent], **kwargs):
    runtime, runtime_dist = 0, []
    a_star_calls_counter, a_star_calls_counter_dist = 0, []
    a_star_n_closed, a_star_n_closed_dist = 0, []
    for agent in agents:

        # create initial plan
        start_time = time.time()
        # info: {'a_s_time': time.time() - start_time, 'a_s_info': a_s_info}
        succeeded, info = agent.calc_a_star_plan(**kwargs)
        # stats
        runtime += time.time() - start_time
        runtime_dist.append(time.time() - start_time)
        a_star_calls_counter += 1
        a_star_n_closed += info['a_s_info']['n_closed']
        a_star_n_closed_dist.append(info['a_s_info']['n_closed'])

        # find_nei
        agent.update_nei(agents, **kwargs)

    func_info = {
        'runtime': runtime,
        'dist_runtime': max(runtime_dist),
        'a_star_calls_counter': a_star_calls_counter,
        'a_star_calls_counter_dist': 1,
        'a_star_n_closed': a_star_n_closed,
        'a_star_n_closed_dist': max(a_star_n_closed_dist)
    }
    return func_info


def all_exchange_k_step_paths(agents: List[KSDSAgent], **kwargs):
    k = kwargs['k']
    runtime, runtime_dist = 0, []

    # exchange paths
    for agent in agents:
        start_time = time.time()
        agent.exchange_paths()
        # stats
        runtime += time.time() - start_time
        runtime_dist.append(time.time() - start_time)

    # check for collisions
    plans = {agent.name: agent.path[:k] for agent in agents}
    there_are_collisions, c_v, c_e, cost = just_check_plans(plans)

    func_info = {
        'runtime': runtime,
        'dist_runtime': max(runtime_dist)
    }
    return there_are_collisions, c_v, c_e, cost, func_info


def all_replan(agents: List[KSDSAgent], **kwargs):
    runtime, runtime_dist = 0, [0]
    a_star_calls_counter, a_star_calls_counter_dist = 0, []
    a_star_n_closed, a_star_n_closed_dist = 0, [0]

    for agent in agents:
        start_time = time.time()
        succeeded, info = agent.replan(**kwargs)
        # stats
        runtime += time.time() - start_time
        runtime_dist.append(time.time() - start_time)
        if len(info) > 0:
            a_star_calls_counter += 1
            a_star_n_closed += info['a_s_info']['n_closed']
            a_star_n_closed_dist.append(info['a_s_info']['n_closed'])

    func_info = {
        'runtime': runtime,
        'dist_runtime': max(runtime_dist),
        'a_star_calls_counter': a_star_calls_counter,
        'a_star_calls_counter_dist': 1,
        'a_star_n_closed': a_star_n_closed,
        'a_star_n_closed_dist': max(a_star_n_closed_dist)
    }
    return func_info


def all_move_k_steps(agents: List[KSDSAgent], **kwargs):
    all_paths_are_finished_list = []
    for agent in agents:
        agent_is_finished = agent.update_full_path(**kwargs)
        all_paths_are_finished_list.append(agent_is_finished)

    all_paths_are_finished = all(all_paths_are_finished_list)
    func_info = {}
    return all_paths_are_finished, func_info


def all_cut_full_paths(agents: List[KSDSAgent], **kwargs):
    for agent in agents:
        agent.cut_back_full_path()


def run_k_sds(start_nodes, goal_nodes, nodes, nodes_dict, h_func, **kwargs):
    runtime = 0
    if 'k' not in kwargs:
        kwargs['k'] = 10
    k_step_iteration_limit = kwargs['k_step_iteration_limit'] if 'k_step_iteration_limit' in kwargs else 1000
    alg_name = kwargs['alg_name'] if 'alg_name' in kwargs else f'k-SDS'
    iter_limit = kwargs['a_star_iter_limit'] if 'a_star_iter_limit' in kwargs else 1e100
    plotter = kwargs['plotter'] if 'plotter' in kwargs else None
    middle_plot = kwargs['middle_plot'] if 'middle_plot' in kwargs else False
    final_plot = kwargs['final_plot'] if 'final_plot' in kwargs else True
    plot_per = kwargs['plot_per'] if 'plot_per' in kwargs else 10
    map_dim = kwargs['map_dim'] if 'map_dim' in kwargs else None

    # Creating agents
    agents, agents_dict = create_agents(start_nodes, goal_nodes, nodes, nodes_dict, h_func, plotter, middle_plot, iter_limit, map_dim)

    # alg info dict
    alg_info = get_alg_info_dict()

    # Distributed Part
    for k_step_iteration in range(1000000):
        kwargs['k_step_iteration'] = k_step_iteration
        kwargs['small_iteration'] = 0

        func_info = all_plan_and_find_nei(agents, **kwargs)  # agents
        if check_if_limit_is_crossed(func_info, alg_info, **kwargs):
            return None, {'agents': agents, 'success_rate': 0}

        there_are_collisions = True
        while there_are_collisions:
            kwargs['small_iteration'] += 1

            there_are_collisions, c_v, c_e, cost, func_info = all_exchange_k_step_paths(agents, **kwargs)  # agents
            if check_if_limit_is_crossed(func_info, alg_info, **kwargs):
                return None, {'agents': agents, 'success_rate': 0}

            if not there_are_collisions:
                break
            func_info = all_replan(agents, **kwargs)  # agents
            if check_if_limit_is_crossed(func_info, alg_info, **kwargs):
                return None, {'agents': agents, 'success_rate': 0}

        all_paths_are_finished, func_info = all_move_k_steps(agents, **kwargs)  # agents
        if check_if_limit_is_crossed(func_info, alg_info, **kwargs):
            return None, {'agents': agents, 'success_rate': 0}

        full_plans = {agent.name: agent.full_path for agent in agents}
        iteration_print(agents, full_plans, alg_name, alg_info, runtime, k_step_iteration)
        if all_paths_are_finished:
            # there_is_col_0, c_v_0, c_e_0, cost_0 = just_check_plans(full_plans)
            all_cut_full_paths(agents, **kwargs)
            cut_full_plans = {agent.name: agent.full_path for agent in agents}
            there_is_col, c_v, c_e, cost = just_check_plans(cut_full_plans)
            if there_is_col:
                raise RuntimeError('uff')
            else:
                if final_plot:
                    print(f'#########################################################')
                    print(f'#########################################################')
                    print(f'#########################################################')
                    plotter.plot_mapf_paths(paths_dict=cut_full_plans, nodes=nodes, plot_per=plot_per)
                alg_info['success_rate'] = 1
                alg_info['sol_quality'] = cost
                alg_info['runtime'] = runtime
                alg_info['a_star_calls_per_agent'] = [agent.stats_n_calls for agent in agents]
                alg_info['n_messages_per_agent'] = [agent.stats_n_messages for agent in agents]
                alg_info['confs_per_iter'] = np.sum([agent.stats_confs_per_iter for agent in agents], 0).tolist()
            return cut_full_plans, alg_info

        if k_step_iteration > k_step_iteration_limit:
            break

    return None, {'agents': agents, 'success_rate': 0}


def main():
    # random_seed = True
    random_seed = False
    seed = 277
    n_agents = 200
    PLOT_PER = 1

    to_use_profiler = True
    # to_use_profiler = False

    k = 7
    # DECISION_TYPE = 'simple'
    DECISION_TYPE = 'max_prev'

    # img_dir = 'empty-48-48.map'  # 48-48
    # img_dir = 'random-64-64-10.map'  # 64-64
    img_dir = 'warehouse-10-20-10-2-1.map'  # 63-161
    # img_dir = 'lt_gallowstemplar_n.map'  # 180-251

    profiler = cProfile.Profile()
    if to_use_profiler:
        profiler.enable()
    for i in range(3):
        print(f'\n[run {i}]')
        result, info = test_mapf_alg_from_pic(
            algorithm=run_k_sds,
            img_dir=img_dir,
            alg_name='k-SDS',
            k=k,
            decision_type=DECISION_TYPE,
            a_star_func=a_star,
            n_agents=n_agents,
            random_seed=random_seed,
            seed=seed,
            final_plot=True,
            a_star_iter_limit=5e7,
            # limit_type='norm_time',
            limit_type='dist_time',
            max_time=5,
            a_star_closed_nodes_limit=1e6,
            plot_per=PLOT_PER,
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
