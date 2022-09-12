import random

import matplotlib.pyplot as plt

from alg_a_star import a_star
from test_mapf_alg import test_mapf_alg_from_pic
from metrics import check_for_collisions, c_v_check_for_agent, c_e_check_for_agent, build_constraints


class DSAgent:
    def __init__(self, index, start_node, goal_node, nodes, nodes_dict, h_func, plotter, middle_plot):
        self.index = index
        self.name = f'agent_{index}'
        self.start_node = start_node
        self.goal_node = goal_node
        self.nodes = nodes
        self.nodes_dict = nodes_dict
        self.h_func = h_func
        self.plotter = plotter
        self. middle_plot = middle_plot
        self.path = []
        self.other_paths = {}

    def exchange(self, agents):
        self.other_paths = {agent.name: agent.path for agent in agents if agent.name != self.name}

    def plan(self):
        sub_results = {k: v for k, v in self.other_paths.items() if len(v) > 0}
        c_v_list = c_v_check_for_agent(self.name, self.path, sub_results)
        c_e_list = c_e_check_for_agent(self.name, self.path, sub_results)

        v_constr_dict, e_constr_dict, perm_constr_dict = build_constraints(self.nodes, sub_results)

        new_path = a_star(start=self.start_node, goal=self.goal_node, nodes=self.nodes,
                          h_func=self.h_func, v_constr_dict=v_constr_dict, e_constr_dict=e_constr_dict,
                          perm_constr_dict=perm_constr_dict,
                          plotter=self.plotter, middle_plot=self.middle_plot)

        if new_path is not None:
            c_v_list_after = c_v_check_for_agent(self.name, new_path, sub_results)
            c_e_list_after = c_e_check_for_agent(self.name, new_path, sub_results)
            if len(c_v_list_after) > 0 or len(c_e_list_after) > 0:
                raise RuntimeError('a_star failed')

            if random.random() < 0.8:
                self.path = new_path

            return True
        return False


def run_ds_mapf(start_nodes, goal_nodes, nodes, nodes_dict, h_func, plotter=None, middle_plot=False, **kwargs):
    # Creating agents
    agents = []
    n_agent = 0
    for start_node, goal_node in zip(start_nodes, goal_nodes):
        agent = DSAgent(n_agent, start_node, goal_node, nodes, nodes_dict, h_func, plotter, middle_plot)
        agents.append(agent)
        n_agent += 1

    # Distributed Part
    for iteration in range(100):

        for agent in agents:
            agent.plan()

        for agent in agents:
            agent.exchange(agents=agents)

        plan = {agent.name: agent.path for agent in agents}
        plan_lngths = [len(path) for path in plan.values()]

        if 0 in plan_lngths:
            print(f'\r---\n[iter {iteration}]\n---\n')
        else:
            cost = sum([len(path) for path in plan.values()])
            there_is_col, c_v, c_e = check_for_collisions(plan)
            print(f'\r---\n[iter {iteration}] \ncost: {cost}\n---\n')
            if not there_is_col:
                print(f'#########################################################')
                print(f'#########################################################')
                print(f'#########################################################')
                plotter.plot_mapf_paths(paths_dict=plan, nodes=nodes)
                return plan, {'agents': agents}

    # partial order
    pass

    return None, {'agents': agents}


def main():
    for i in range(20):
        print(f'\n[run {i}]')
        result, info = test_mapf_alg_from_pic(algorithm=run_ds_mapf, initial_ordering=[], n_agents=n_agents,
                                              random_seed=random_seed, seed=seed)

        if not random_seed:
            break

        # plt.show()
        plt.close()


if __name__ == '__main__':
    random_seed = True
    # random_seed = False
    seed = 197
    n_agents = 10

    # good example: img_png = '19_20_warehouse.png'
    # random_seed = True
    # random_seed = False
    # seed = 23
    # n_agents = 30

    main()

