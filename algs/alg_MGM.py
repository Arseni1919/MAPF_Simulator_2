import random
import time
import matplotlib.pyplot as plt
import cProfile
import pstats
from algs.test_mapf_alg import test_mapf_alg_from_pic

from algs.metrics import check_for_collisions, c_v_check_for_agent, c_e_check_for_agent
from algs.metrics import build_constraints
from algs.metrics import crossed_time_limit


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

    def exchange_paths(self, agents):
        self.other_paths = {agent.name: agent.path for agent in agents if agent.name != self.name}

    def plan(self):
        pass

    def update_gain(self):
        pass

    def exchange_gains(self, agents):
        self.other_gains = {agent.name: agent.gain for agent in agents if agent.name != self.name}

    def decision_bool(self):
        pass

    def take_decision(self):
        pass


def run_mgm(start_nodes, goal_nodes, nodes, nodes_dict, h_func, **kwargs):
    start_time = time.time()
    a_star_calls_limit = kwargs['a_star_calls_limit'] if 'a_star_calls_limit' in kwargs else 1e100
    max_time = kwargs['max_time'] if 'max_time' in kwargs else 60
    final_plot = kwargs['final_plot'] if 'final_plot' in kwargs else True
    plot_per = kwargs['plot_per'] if 'plot_per' in kwargs else 10
    plotter = kwargs['plotter'] if 'plotter' in kwargs else None

    middle_plot = kwargs['middle_plot'] if 'middle_plot' in kwargs else False
    a_star_iter_limit = kwargs['a_star_iter_limit'] if 'a_star_iter_limit' in kwargs else 1e100
    map_dim = kwargs['map_dim'] if 'map_dim' in kwargs else None
    limit_type = kwargs['limit_type'] if 'limit_type' in kwargs else 'simple'

    plan = None

    # Creating agents
    agents = []
    n_agent = 0
    for start_node, goal_node in zip(start_nodes, goal_nodes):
        agent = MGMAgent(n_agent, start_node, goal_node, nodes, nodes_dict, h_func, **kwargs)
        agents.append(agent)
        n_agent += 1

    alg_info = {'agents': agents,
                'success_rate': 0,
                'sol_quality': 0,
                'runtime': (time.time() - start_time),
                'iterations_time': 0,
                'a_star_calls_counter': 0,
                'a_star_calls_dist_counter': 0,
                'a_star_runtimes': [],
                'a_star_n_closed': [],
                'n_agents_conf': []}

    # ITERATIONS
    for iteration in range(1000000):

        # LIMITS
        if crossed_time_limit(start_time, max_time):
            break
        if alg_info['a_star_calls_counter'] >= a_star_calls_limit:
            break

        # PLAN
        for agent in agents:
            agent.plan()

        # EXCHANGE PATHS
        for agent in agents:
            agent.exchange_paths(agents=agents)

        # UPDATE GAIN
        for agent in agents:
            agent.update_gain()

        # EXCHANGE GAINS
        for agent in agents:
            agent.exchange_gains(agents=agents)

        # DECISION
        for agent in agents:
            agent.take_decision()

        # CHECK PLAN
        plan = {agent.name: agent.path for agent in agents}
        plan_lngths = [len(path) for path in plan.values()]
        if 0 in plan_lngths:
            raise RuntimeError('0 in plan_lngths')
        cost = sum([len(path) for path in plan.values()])
        print(f'\r---\n'
              f'[MGM][{len(agents)} agents][A* calls: ][A* dist calls: ][time: {time.time() - start_time:0.2f}s][iter {iteration}]\n'
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
    n_agents = 50
    A_STAR_ITER_LIMIT = 5e7
    MAX_TIME = 5
    PLOT_PER = 1
    to_use_profiler = True
    main()
