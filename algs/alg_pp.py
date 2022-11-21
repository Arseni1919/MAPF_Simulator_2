import random
import time
import matplotlib.pyplot as plt
import cProfile
import pstats
from algs.test_mapf_alg import test_mapf_alg_from_pic

# from algs.metrics import check_for_collisions, c_v_check_for_agent, c_e_check_for_agent
from algs.metrics import build_constraints, get_agents_in_conf, check_plan, get_alg_info_dict
from algs.metrics import crossed_time_limit
from algs.alg_a_star import a_star


class PPAgent:
    def __init__(self, index: int, start_node, goal_node):
        self.index = index
        self.name = f'agent_{index}'
        self.start_node = start_node
        self.start_xy = self.start_node.xy_name
        self.goal_node = goal_node
        self.goal_xy = self.goal_node.xy_name
        self.path = []


def create_agents(start_nodes, goal_nodes):
    agents, agents_dict = [], {}
    index = 0
    for start_node, goal_node in zip(start_nodes, goal_nodes):
        new_agent = PPAgent(index, start_node, goal_node)
        agents.append(new_agent)
        agents_dict[new_agent.name] = new_agent
        index += 1
    return agents, agents_dict


def update_path(update_agent, higher_agents, nodes, nodes_dict, h_func):
    # print('\rFUNC: update_path', end='')
    sub_results = {agent.name: agent.path for agent in higher_agents}
    v_constr_dict, e_constr_dict, perm_constr_dict = build_constraints(nodes, sub_results)
    # print('\rBEFORE A*', end='')
    print(f'\n ---------- (PP) A* {update_agent.name} ---------- \n')
    new_path, a_s_info = a_star(start=update_agent.start_node, goal=update_agent.goal_node,
                                nodes=nodes, nodes_dict=nodes_dict,
                                h_func=h_func,
                                v_constr_dict=v_constr_dict,
                                e_constr_dict=e_constr_dict,
                                perm_constr_dict=perm_constr_dict)
    return new_path, a_s_info


def run_pp(start_nodes, goal_nodes, nodes, nodes_dict, h_func, **kwargs):
    start_time = time.time()
    a_star_calls_limit = kwargs['a_star_calls_limit'] if 'a_star_calls_limit' in kwargs else 1e100
    max_time = kwargs['max_time'] if 'max_time' in kwargs else 60
    plotter = kwargs['plotter'] if 'plotter' in kwargs else None
    final_plot = kwargs['final_plot'] if 'final_plot' in kwargs else True
    plot_per = kwargs['plot_per'] if 'plot_per' in kwargs else 10

    agents, agents_dict = create_agents(start_nodes, goal_nodes)
    agent_names = [agent.name for agent in agents]

    plan = None
    alg_info = get_alg_info_dict()

    # ITERATIONS
    for iteration in range(1000000):

        # LIMITS
        if crossed_time_limit(start_time, max_time):
            break
        if alg_info['a_star_calls_counter'] >= a_star_calls_limit:
            break

        # PICK A RANDOM ORDE
        random.shuffle(agent_names)
        new_order = [agents_dict[agent_name] for agent_name in agent_names]

        # PLAN
        higher_agents = []
        to_continue = False
        for agent in agents:
            new_path, a_s_info = update_path(agent, higher_agents, nodes, nodes_dict, h_func)
            alg_info['a_star_calls_counter'] += 1
            alg_info['a_star_runtimes'].append(time.time() - start_time)
            alg_info['a_star_n_closed'].append(a_s_info['n_closed'])
            if new_path:
                agent.path = new_path
            else:
                to_continue = True
                break
            higher_agents.append(agent)
        if to_continue:
            continue

        # CHECK PLAN
        plan = {agent.name: agent.path for agent in agents}
        there_is_col, c_v, c_e, cost = check_plan(agents, plan, 'PP', alg_info, start_time, iteration)
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
            algorithm=run_pp,
            n_agents=n_agents,
            random_seed=random_seed,
            seed=seed,
            final_plot=True,
            a_star_iter_limit=A_STAR_ITER_LIMIT,
            a_star_calls_limit=A_STAR_CALLS_LIMIT,
            max_time=MAX_TIME,
            plot_per=PLOT_PER,
        )

        if not random_seed:
            break

        # plt.show()
        plt.close()

    if to_use_profiler:
        profiler.disable()
        stats = pstats.Stats(profiler).sort_stats('cumtime')
        stats.dump_stats('../stats/results_pp.pstat')


if __name__ == '__main__':
    random_seed = True
    # random_seed = False
    seed = 277
    n_agents = 50
    A_STAR_ITER_LIMIT = 5e7
    A_STAR_CALLS_LIMIT = 1000
    MAX_TIME = 5
    PLOT_PER = 10
    to_use_profiler = True
    main()
