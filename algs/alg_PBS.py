import copy
import random
import numpy as np

import matplotlib.pyplot as plt

from alg_a_star import a_star
from test_mapf_alg import test_mapf_alg_from_pic
from metrics import check_for_collisions, c_v_check_for_agent, c_e_check_for_agent


def preprint_func_name(func):
    def inner(*args, **kwargs):
        print(f'\rFUNC: {func.__name__}', end='')
        return func(*args, **kwargs)
    return inner


class PBSAgent:
    def __init__(self, index: int, start_node, goal_node):
        self.index = index
        self.name = f'agent_{index}'
        self.start_node = start_node
        self.start_xy = self.start_node.xy_name
        self.goal_node = goal_node
        self.goal_xy = self.goal_node.xy_name


class PBSNode:
    def __init__(self, agents, agents_dict, index):
        self.agent = agents
        self.agent_dict = agents_dict
        self.index = index
        self.name = f'PBSNode {index}'
        self.plan = {agent.name: [] for agent in agents}
        self.cost = -1
        self.constraints = {agent.name: [] for agent in agents}
        self.ordering_rules = []
        self.partial_order = []
        self.parent = None

    def calc_cost(self):
        self.cost = sum([len(path) for path in self.plan.values()])

    def update_ordering_rules(self):
        if len(self.partial_order) > 0:
            self.ordering_rules = []
            for agent_index, agent_name in enumerate(self.partial_order):
                for i in range(agent_index):
                    self.ordering_rules.append((self.partial_order[i], agent_name))


def create_agents(start_nodes, goal_nodes):
    agents, agents_dict = [], {}
    index = 0
    for start_node, goal_node in zip(start_nodes, goal_nodes):
        new_agent = PBSAgent(index, start_node, goal_node)
        agents.append(new_agent)
        agents_dict[new_agent.name] = new_agent
        index += 1
    return agents, agents_dict


def get_order_lists(pbs_node, agent):
    if agent.name in pbs_node.partial_order:
        agent_index = pbs_node.partial_order.index(agent.name)
        higher_order_names = [pbs_node.partial_order[i] for i in range(0, agent_index)]
        lower_order_names = [pbs_node.partial_order[i] for i in range(agent_index + 1, len(pbs_node.partial_order))]
        higher_order_list = [pbs_node.agent_dict[name] for name in higher_order_names]
        lower_order_list = [pbs_node.agent_dict[name] for name in lower_order_names]
        return higher_order_list, lower_order_list
    return [], []


# @preprint_func_name
def topological_sorting(pbs_node, agent):
    print('\rFUNC: topological_sorting', end='')
    update_list = [agent]
    h_l, lower_order_list = get_order_lists(pbs_node, agent)
    if len(lower_order_list) > 0:
        lower_order_names = [a.name for a in lower_order_list]
        lower_order_dict = {a.name: a for a in lower_order_list}
        for partial_order_agent in pbs_node.partial_order:
            if partial_order_agent in lower_order_names:
                update_list.append(lower_order_dict[partial_order_agent])
    return update_list


def collide_check(pbs_node, update_agent):
    print('\rFUNC: collide_check', end='')
    higher_order_list, _ = get_order_lists(pbs_node, update_agent)
    sub_results = {agent.name: pbs_node.plan[agent.name] for agent in higher_order_list}
    c_v_list = c_v_check_for_agent(update_agent.name, pbs_node.plan[update_agent.name], sub_results)
    if len(c_v_list) > 0:
        return True
    e_v_list = c_e_check_for_agent(update_agent.name, pbs_node.plan[update_agent.name], sub_results)
    if len(e_v_list):
        return True
    return False


def update_path(pbs_node, update_agent, nodes, nodes_dict, h_func, plotter, middle_plot):
    print('\rFUNC: update_path', end='')
    higher_order_list, _ = get_order_lists(pbs_node, update_agent)
    sub_results = {agent.name: pbs_node.plan[agent.name] for agent in higher_order_list}

    c_v_list = c_v_check_for_agent(update_agent.name, pbs_node.plan[update_agent.name], sub_results)
    c_e_list = c_e_check_for_agent(update_agent.name, pbs_node.plan[update_agent.name], sub_results)

    v_constr_dict = {node.xy_name: [] for node in nodes}
    e_constr_dict = {node.xy_name: [] for node in nodes}
    perm_constr_dict = {node.xy_name: [] for node in nodes}

    for agent_name, path in sub_results.items():
        if len(path) > 0:
            final_node = path[-1]
            perm_constr_dict[final_node.xy_name].append(final_node.t)

            prev_node = path[0]
            for node in path:
                # vertex
                v_constr_dict[f'{node.x}_{node.y}'].append(node.t)
                # edge
                if prev_node.xy_name != node.xy_name:
                    e_constr_dict[f'{prev_node.x}_{prev_node.y}'].append((node.x, node.y, node.t))
                prev_node = node

    print('\rBEFORE A*', end='')
    new_path = a_star(start=update_agent.start_node, goal=update_agent.goal_node, nodes=nodes,
                      h_func=h_func, v_constr_dict=v_constr_dict, e_constr_dict=e_constr_dict, perm_constr_dict=perm_constr_dict,
                      plotter=plotter, middle_plot=middle_plot)

    if new_path is not None:
        c_v_list_after = c_v_check_for_agent(update_agent.name, new_path, sub_results)
        c_e_list_after = c_e_check_for_agent(update_agent.name, new_path, sub_results)
        if len(c_v_list_after) > 0 or len(c_e_list_after) > 0:
            raise RuntimeError('a_star failed')

    return new_path


# @preprint_func_name
def update_plan(pbs_node, agent, nodes, nodes_dict, h_func, plotter, middle_plot):
    print('\rFUNC: update_plan', end='')
    update_list = topological_sorting(pbs_node, agent)

    for update_agent in update_list:
        if collide_check(pbs_node, update_agent) or update_agent.name == agent.name:
            new_path = update_path(pbs_node, update_agent, nodes, nodes_dict, h_func, plotter, middle_plot)
            if new_path is None:
                return False
            pbs_node.plan[update_agent.name] = new_path
    return True


def choose_conf(c_v, c_e):
    if len(c_v) > 0:
        # random.shuffle(c_v)
        return c_v[0], 'vertex'
    elif len(c_e) > 0:
        # random.shuffle(c_e)
        return c_e[0], 'edge'
    else:
        raise RuntimeError('no collisions')


# @preprint_func_name
def add_new_ordering(NEW_pbs_node, NEXT_pbs_node, agent, conf):
    print('\rFUNC: add_new_ordering', end='')
    h_agent, l_agent = None, None
    new_name = None
    if agent.name == conf[0]:
        h_agent = agent.name
        l_agent = conf[1]
    elif agent.name == conf[1]:
        h_agent = conf[0]
        l_agent = agent.name
    else:
        raise RuntimeError('no such name in constr')

    NEW_pbs_node.partial_order = copy.deepcopy(NEXT_pbs_node.partial_order)
    if h_agent in NEW_pbs_node.partial_order and l_agent in NEW_pbs_node.partial_order:
        raise RuntimeError('a star did something wrong so')
    if h_agent in NEW_pbs_node.partial_order:
        h_index = NEW_pbs_node.partial_order.index(h_agent)
        NEW_pbs_node.partial_order.insert(h_index + 1, l_agent)
        new_name = l_agent
    elif l_agent in NEW_pbs_node.partial_order:
        l_index = NEW_pbs_node.partial_order.index(l_agent)
        NEW_pbs_node.partial_order.insert(l_index, h_agent)
        new_name = h_agent
    else:
        NEW_pbs_node.partial_order.append(agent.name)
        new_name = agent.name

    NEW_pbs_node.update_ordering_rules()

    for (i_1, i_2) in NEW_pbs_node.ordering_rules:
        if (i_2, i_1) in NEW_pbs_node.ordering_rules:
            raise RuntimeError('there are upsidedown orders')

    return NEW_pbs_node.agent_dict[new_name]


def run_pbs(start_nodes, goal_nodes, nodes, nodes_dict, h_func, plotter=None, middle_plot=False, **kwargs):
    partial_order = kwargs['initial_ordering']
    pbs_node_index = 0
    agents, agents_dict = create_agents(start_nodes, goal_nodes)
    root = PBSNode(agents, agents_dict, pbs_node_index)
    root.partial_order = partial_order
    root.update_ordering_rules()

    for agent in agents:
        success = update_plan(root, agent, nodes, nodes_dict, h_func, plotter, middle_plot)
        if not success:
            return None, {}

    root.calc_cost()
    stack = [root]
    iteration = 0
    while len(stack) > 0:
        iteration += 1
        NEXT_pbs_node = stack.pop()
        there_is_col, c_v, c_e = check_for_collisions(NEXT_pbs_node.plan)
        print(f'\r---\n[iter {iteration}] PBS Node {NEXT_pbs_node.index}, stack: {len(stack)}\norder: {NEXT_pbs_node.partial_order}\ncost: {NEXT_pbs_node.cost}\n---\n')

        if not there_is_col:
            print(f'#########################################################')
            print(f'#########################################################')
            print(f'#########################################################')
            plotter.plot_mapf_paths(paths_dict=NEXT_pbs_node.plan, nodes=nodes)
            return NEXT_pbs_node.plan, {'PBSNode': NEXT_pbs_node}

        conf, conf_type = choose_conf(c_v, c_e)
        for i in range(2):
            pbs_node_index += 1
            NEW_pbs_node = PBSNode(agents, agents_dict, pbs_node_index)
            NEW_pbs_node.plan = copy.deepcopy(NEXT_pbs_node.plan)

            # NEW_pbs_node.constraints = copy.deepcopy(NEXT_pbs_node.constraints)
            # add_new_constraint(NEW_pbs_node, i, conf, conf_type)

            agent = NEXT_pbs_node.agent_dict[conf[i]]
            agent = add_new_ordering(NEW_pbs_node, NEXT_pbs_node, agent, conf)
            success = update_plan(NEW_pbs_node, agent, nodes, nodes_dict, h_func, plotter, middle_plot)

            if success:
                NEW_pbs_node.calc_cost()
                stack.append(NEW_pbs_node)
                NEW_pbs_node.parent = NEXT_pbs_node
                stack.sort(key=lambda x: x.cost, reverse=True)

    return None, {}


def main():
    # initial_ordering = [f'agent_{n}' for n in range(n_agents)]
    # result = test_mapf_alg_from_pic(algorithm=run_pbs, initial_ordering=initial_ordering, n_agents=n_agents)
    for i in range(20):
        print(f'\n[run {i}]')
        result, info = test_mapf_alg_from_pic(algorithm=run_pbs, initial_ordering=[], n_agents=n_agents)

        # plt.show()
        plt.close()


if __name__ == '__main__':
    random_seed = True
    # random_seed = False
    seed = 667
    n_agents = 7

    if random_seed:
        seed = random.choice(range(1000))

    print(f'SEED: {seed}')

    random.seed(seed)
    np.random.seed(seed)

    main()
