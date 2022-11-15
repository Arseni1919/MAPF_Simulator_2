import random
import time
import matplotlib.pyplot as plt
import cProfile
import pstats
from algs.test_mapf_alg import test_mapf_alg_from_pic

from algs.metrics import check_for_collisions, c_v_check_for_agent, c_e_check_for_agent
from algs.metrics import build_constraints, get_agents_in_conf
from algs.metrics import crossed_time_limit
from algs.alg_a_star import a_star


class MGMAgent:
    def __init__(self, n_agent, start_node, goal_node, nodes, nodes_dict, h_func, **kwargs):
        self.index = n_agent
        self.start_node = start_node
        self.goal_node = goal_node
        self.nodes = nodes
        self.nodes_dict = nodes_dict
        self.h_func = h_func
        self.name = f'agent_{self.index}'
        self.path = []
        self.gain = 0
        self.other_paths = {}
        self.other_gains = {}
        self.agents_in_confs = []

    def exchange_paths(self, agents):
        self.other_paths = {agent.name: agent.path for agent in agents if agent.name != self.name}
        c_v_list = c_v_check_for_agent(self.name, self.path, self.other_paths)
        c_e_list = c_e_check_for_agent(self.name, self.path, self.other_paths)
        self.gain = len(c_v_list) + len(c_e_list)
        self.agents_in_confs = get_agents_in_conf(c_v_list, c_e_list)

    def plan(self, alg_info, initial=False):
        start_time = time.time()
        v_constr_dict, e_constr_dict, perm_constr_dict = build_constraints(self.nodes, self.other_paths)
        # iter_limit = self.get_a_star_iter_limit(agents_in_confs)
        print(f'\n ---------- (MGM) A* {self.name} ---------- \n')
        new_path, a_s_info = a_star(start=self.start_node, goal=self.goal_node,
                                    nodes=self.nodes, nodes_dict=self.nodes_dict, h_func=self.h_func,
                                    v_constr_dict=v_constr_dict,
                                    e_constr_dict=e_constr_dict,
                                    perm_constr_dict=perm_constr_dict)
        if new_path is not None:
            self.path = new_path
        if self.path is None:
            raise RuntimeError('self.path is None')

        # stats
        alg_info['a_star_calls_counter'] += 1
        alg_info['a_star_runtimes'].append(time.time() - start_time)
        alg_info['a_star_n_closed'].append(a_s_info['n_closed'])
        if not initial:
            alg_info['n_agents_conf'].append(len(self.agents_in_confs))
        # return {'elapsed': time.time() - start_time, 'a_s_info': a_s_info, 'n_agents_conf': len(agents_in_confs)}

    def exchange_gains(self, agents):
        self.other_gains = {agent.name: agent.gain for agent in agents if agent.name != self.name}

    def decision_bool(self, agents_dict):
        if len(self.agents_in_confs) == 0:
            return False
        equal_agents_indecies = []
        other_nei_gains = {
            agent_name: gain for agent_name, gain in self.other_gains.items() if agent_name in self.agents_in_confs
        }
        for agent_name, gain in other_nei_gains.items():
            if self.gain < gain:
                return False
            if self.gain == gain:
                equal_agents_indecies.append(agents_dict[agent_name].index)

        if len(equal_agents_indecies) > 0 and min(equal_agents_indecies) < self.index:
            return False
        return True

    def take_decision(self, agents_dict, alg_info):
        # take max value
        if self.decision_bool(agents_dict):
            self.plan(alg_info)
        # else:
        #     print(f'\n ---------- (MGM) NO NEED FOR A* {self.name} ---------- \n')


def run_mgm(start_nodes, goal_nodes, nodes, nodes_dict, h_func, **kwargs):
    start_time = time.time()
    a_star_calls_limit = kwargs['a_star_calls_limit'] if 'a_star_calls_limit' in kwargs else 1e100
    max_time = kwargs['max_time'] if 'max_time' in kwargs else 60
    plotter = kwargs['plotter'] if 'plotter' in kwargs else None
    final_plot = kwargs['final_plot'] if 'final_plot' in kwargs else True
    plot_per = kwargs['plot_per'] if 'plot_per' in kwargs else 10

    middle_plot = kwargs['middle_plot'] if 'middle_plot' in kwargs else False
    a_star_iter_limit = kwargs['a_star_iter_limit'] if 'a_star_iter_limit' in kwargs else 1e100
    map_dim = kwargs['map_dim'] if 'map_dim' in kwargs else None
    limit_type = kwargs['limit_type'] if 'limit_type' in kwargs else 'simple'

    plan = None

    # Creating agents
    agents = []
    agents_dict = {}
    n_agent = 0
    for start_node, goal_node in zip(start_nodes, goal_nodes):
        agent = MGMAgent(n_agent, start_node, goal_node, nodes, nodes_dict, h_func, **kwargs)
        agents.append(agent)
        agents_dict[agent.name] = agent
        n_agent += 1

    alg_info = {'agents': agents,
                'success_rate': 0,
                'sol_quality': 0,
                'runtime': 0,
                # 'iterations_time': 0,
                'a_star_calls_counter': 0,
                # 'a_star_calls_dist_counter': 0,
                'a_star_runtimes': [],
                'a_star_n_closed': [],
                'n_agents_conf': []}

    # ITERATIONS

    # PLAN
    for agent in agents:
        agent.plan(alg_info, initial=True)

    for iteration in range(1000000):

        # LIMITS
        if crossed_time_limit(start_time, max_time):
            break
        if alg_info['a_star_calls_counter'] >= a_star_calls_limit:
            break

        # EXCHANGE PATHS + UPDATE GAIN
        for agent in agents:
            agent.exchange_paths(agents=agents)

        # EXCHANGE GAINS
        for agent in agents:
            agent.exchange_gains(agents=agents)

        # DECISION
        for agent in agents:
            agent.take_decision(agents_dict, alg_info)

        # CHECK PLAN
        plan = {agent.name: agent.path for agent in agents}
        plan_lngths = [len(path) for path in plan.values()]
        if 0 in plan_lngths:
            raise RuntimeError('0 in plan_lngths')
        cost = sum([len(path) for path in plan.values()])
        print(f'\r---\n'
              f'[MGM][{len(agents)} agents][A* calls: {alg_info["a_star_calls_counter"]}][time: {time.time() - start_time:0.2f}s][iter {iteration}][A* dist calls: ]\n'
              f'cost: {cost}\n'
              f'---\n')

        there_is_col, c_v, c_e = check_for_collisions(plan)
        if not there_is_col:
            if final_plot:
                print(f'#########################################################')
                print(f'#########################################################')
                print(f'#########################################################')
                plotter.plot_mapf_paths(paths_dict=plan, nodes=nodes, plot_per=plot_per)

            alg_info['success_rate'] = 1
            alg_info['sol_quality'] = cost
            alg_info['runtime'] = time.time() - start_time
            return plan, alg_info

    return plan, alg_info


def main():
    profiler = cProfile.Profile()
    if to_use_profiler:
        profiler.enable()

    for i in range(3):
        print(f'\n[run {i}]')
        result, info = test_mapf_alg_from_pic(
            algorithm=run_mgm,
            n_agents=n_agents,
            random_seed=random_seed,
            seed=seed,
            final_plot=True,
            a_star_iter_limit=A_STAR_ITER_LIMIT,
            a_star_calls_limit=A_STAR_CALLS_LIMIT,
            max_time=MAX_TIME,
            plot_per=PLOT_PER,
            limit_type='smart'
        )

        if not random_seed:
            break

        # plt.show()
        plt.close()

    if to_use_profiler:
        profiler.disable()
        stats = pstats.Stats(profiler).sort_stats('cumtime')
        stats.dump_stats('../stats/results_mgm.pstat')


if __name__ == '__main__':
    random_seed = True
    # random_seed = False
    seed = 277
    n_agents = 100
    A_STAR_ITER_LIMIT = 5e7
    A_STAR_CALLS_LIMIT = 1000
    MAX_TIME = 5
    PLOT_PER = 10
    to_use_profiler = True
    main()
