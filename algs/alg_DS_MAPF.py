import random
import time
import matplotlib.pyplot as plt

from algs.alg_a_star import a_star
from algs.test_mapf_alg import test_mapf_alg_from_pic
from algs.metrics import check_for_collisions, c_v_check_for_agent, c_e_check_for_agent, build_constraints, crossed_time_limit


class DSAgent:
    def __init__(self, index, start_node, goal_node, nodes, nodes_dict, h_func, plotter, middle_plot, iter_limit=1e100):
        self.index = index
        self.name = f'agent_{index}'
        self.start_node = start_node
        self.goal_node = goal_node
        self.nodes = nodes
        self.nodes_dict = nodes_dict
        self.h_func = h_func
        self.plotter = plotter
        self. middle_plot = middle_plot
        self.iter_limit = iter_limit
        self.path = []
        self.other_paths = {}
        self.conf_paths = {}

    def exchange(self, agents):
        self.other_paths = {agent.name: agent.path for agent in agents if agent.name != self.name}

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

    def plan(self):
        start_time = time.time()
        sub_results = {k: v for k, v in self.other_paths.items() if len(v) > 0}
        # c_v_list = c_v_check_for_agent(self.name, self.path, self.conf_paths)
        # c_e_list = c_e_check_for_agent(self.name, self.path, self.conf_paths)

        # v_constr_dict, e_constr_dict, perm_constr_dict = build_constraints(self.nodes, self.conf_paths)
        v_constr_dict, e_constr_dict, perm_constr_dict = build_constraints(self.nodes, sub_results)

        new_path = a_star(start=self.start_node, goal=self.goal_node, nodes=self.nodes, h_func=self.h_func,
                          v_constr_dict=v_constr_dict,
                          e_constr_dict=e_constr_dict,
                          perm_constr_dict=perm_constr_dict,
                          plotter=self.plotter, middle_plot=self.middle_plot,
                          iter_limit=self.iter_limit)

        elapsed = time.time() - start_time
        if new_path is not None:
            c_v_list_after_1 = c_v_check_for_agent(self.name, new_path, self.conf_paths)
            c_e_list_after_1 = c_e_check_for_agent(self.name, new_path, self.conf_paths)
            # c_v_list_after_2 = c_v_check_for_agent(self.name, new_path, sub_results)
            # c_e_list_after_2 = c_e_check_for_agent(self.name, new_path, sub_results)
            if len(c_v_list_after_1) > 0 or len(c_e_list_after_1) > 0:
                raise RuntimeError('a_star failed')

            if random.random() < 0.8:
                self.path = new_path

            return True, {'elapsed': elapsed}
        return False, {'elapsed': elapsed}


def run_ds_mapf(start_nodes, goal_nodes, nodes, nodes_dict, h_func, plotter=None, middle_plot=False, **kwargs):
    start_time = time.time()
    iterations_time = 0
    if 'max_time' in kwargs:
        max_time = kwargs['max_time']
    else:
        max_time = 10
    if 'final_plot' in kwargs:
        final_plot = kwargs['final_plot']
    else:
        final_plot = True
    if 'a_star_iter_limit' in kwargs:
        iter_limit = kwargs['a_star_iter_limit']
    else:
        iter_limit = 1e100
    # Creating agents
    agents = []
    n_agent = 0
    for start_node, goal_node in zip(start_nodes, goal_nodes):
        agent = DSAgent(n_agent, start_node, goal_node, nodes, nodes_dict, h_func, plotter, middle_plot, iter_limit)
        agents.append(agent)
        n_agent += 1

    # Distributed Part
    for iteration in range(100000):
        max_time_list = []
        # time constraint
        # if crossed_time_limit(start_time, max_time * len(agents)):
        if crossed_time_limit(start_time, max_time):
            break

        for agent in agents:
            succeeded, info = agent.plan()
            max_time_list.append(info['elapsed'])
        iterations_time += max(max_time_list)

        for agent in agents:
            agent.exchange(agents=agents)

        plan = {agent.name: agent.path for agent in agents}
        plan_lngths = [len(path) for path in plan.values()]

        if 0 in plan_lngths:
            print(f'\r---\n[DS-MAPF][{len(agents)} agents][time: {time.time() - start_time:0.2f}s][iter {iteration}]\n---\n')
        else:
            cost = sum([len(path) for path in plan.values()])
            there_is_col, c_v, c_e = check_for_collisions(plan)
            print(f'\r---\n[iter {iteration}] \ncost: {cost}\n---\n')
            if not there_is_col:
                if final_plot:
                    print(f'#########################################################')
                    print(f'#########################################################')
                    print(f'#########################################################')
                    plotter.plot_mapf_paths(paths_dict=plan, nodes=nodes, plot_per=1)
                return plan, {'agents': agents,
                              'success_rate': 1,
                              'sol_quality': cost,
                              'runtime': (time.time() - start_time),
                              'iterations_time': iterations_time}

    # partial order
    pass

    return None, {'agents': agents, 'success_rate': 0}


def main():
    for i in range(20):
        print(f'\n[run {i}]')
        result, info = test_mapf_alg_from_pic(algorithm=run_ds_mapf, initial_ordering=[], n_agents=n_agents,
                                              random_seed=random_seed, seed=seed, final_plot=True)

        if not random_seed:
            break

        # plt.show()
        plt.close()


if __name__ == '__main__':
    random_seed = True
    # random_seed = False
    seed = 183
    n_agents = 5

    # good example: img_png = '19_20_warehouse.png'
    # random_seed = True
    # random_seed = False
    # seed = 23
    # n_agents = 30

    main()


