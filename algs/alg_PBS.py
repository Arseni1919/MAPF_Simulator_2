import copy

import matplotlib.pyplot as plt

from alg_a_star import a_star
from test_mapf_alg import test_mapf_alg_from_pic
from metrics import check_for_collisions, c_v_check_for_agent, c_e_check_for_agent


class PBSAgent:
    def __init__(self, index, start_node, goal_node):
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
        self.ordering = []

    def calc_cost(self):
        self.cost = sum([len(path) for path in self.plan.values()])


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
    higher_order_list = []
    lower_order_list = []
    for order in pbs_node.ordering:
        higher_a, lower_a = order
        if higher_a == agent.name:
            lower_order_list.append(pbs_node.agent_dict[lower_a])
        if lower_a == agent.name:
            higher_order_list.append(pbs_node.agent_dict[higher_a])

    return higher_order_list, lower_order_list


def topological_sorting(pbs_node, agent):
    update_list = [agent]
    h_l, lower_order_list = get_order_lists(pbs_node, agent)

    while len(lower_order_list) != 0:
        agent_1 = lower_order_list.pop(0)
        for agent_2 in lower_order_list:
            # no higher order in the list of orders
            if (agent_2.name, agent_1.name) in pbs_node.ordering:
                lower_order_list.append(agent_1)
                break
        update_list.append(agent_1)

    # update_list.extend(lower_order_list)

    return update_list


def collide_check(pbs_node, update_agent):
    higher_order_list, _ = get_order_lists(pbs_node, update_agent)
    sub_results = {agent.name: pbs_node.plan[agent.name] for agent in higher_order_list}
    c_v_list = c_v_check_for_agent(update_agent, pbs_node.plan[update_agent.name], sub_results)
    if len(c_v_list) > 0:
        return True
    e_v_list = c_e_check_for_agent(update_agent, pbs_node.plan[update_agent.name], sub_results)
    if len(e_v_list):
        return True
    return False


def update_path(pbs_node, update_agent, nodes, nodes_dict, h_func, plotter, middle_plot):
    # print('FUNC: update_path')
    higher_order_list, _ = get_order_lists(pbs_node, update_agent)
    sub_results = {agent.name: pbs_node.plan[agent.name] for agent in higher_order_list}
    c_v_list = c_v_check_for_agent(update_agent, pbs_node.plan[update_agent.name], sub_results)
    c_e_list = c_e_check_for_agent(update_agent, pbs_node.plan[update_agent.name], sub_results)
    constraint_dict = {node.xy_name: [] for node in nodes}
    for constr in c_v_list:
        agent_1, agent_2, x, y, t = constr
        constraint_dict[f'{x}_{y}'].append(t)
    for constr in c_e_list:
        agent_1, agent_2, prev_x, prev_y, x, y, t = constr
        constraint_dict[f'{x}_{y}'].append(t)
        constraint_dict[f'{prev_x}_{prev_y}'].append(t)
    for constr in pbs_node.constraints[update_agent.name]:
        x, y, t = constr
        constraint_dict[f'{x}_{y}'].append(t)


    # print('BEFORE A*')
    new_path = a_star(start=update_agent.start_node, goal=update_agent.goal_node, nodes=nodes,
                      h_func=h_func, constraint_dict=constraint_dict,
                      plotter=plotter, middle_plot=middle_plot)
    return new_path


def update_plan(pbs_node, agent, nodes, nodes_dict, h_func, plotter, middle_plot):
    print('FUNC: update_plan')
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
        return c_v[0], 'vertex'
    elif len(c_e) > 0:
        return c_e[0], 'edge'
    else:
        raise RuntimeError('no collisions')


def add_new_constraint(pbs_node, index, conf, conf_type):
    print('FUNC: add_new_constraint')
    agent_name = conf[index]
    if conf_type == 'vertex':
        agent_1, agent_2, x, y, t = conf
        pbs_node.constraints[agent_name].append((x, y, t))
    elif conf_type == 'edge':
        agent_1, agent_2, prev_x, prev_y, x, y, t = conf
        pbs_node.constraints[agent_name].append((x, y, t))
        pbs_node.constraints[agent_name].append((prev_x, prev_y, t))
    else:
        raise RuntimeError('type error')


def add_new_ordering(NEW_pbs_node, NEXT_pbs_node, i, conf):
    print('FUNC: add_new_ordering')
    agent_1, agent_2 = conf[0], conf[1]
    NEW_pbs_node.ordering = copy.deepcopy(NEXT_pbs_node.ordering)
    if i == 0:
        NEW_pbs_node.ordering.append((agent_2, agent_1))
    elif i == 1:
        NEW_pbs_node.ordering.append((agent_1, agent_2))
    else:
        raise RuntimeError('i is wrong')


def run_pbs(start_nodes, goal_nodes, nodes, nodes_dict, h_func, plotter=None, middle_plot=False, **kwargs):
    pbs_ordering = kwargs['initial_ordering']
    pbs_node_index = 0
    agents, agents_dict = create_agents(start_nodes, goal_nodes)
    root = PBSNode(agents, agents_dict, pbs_node_index)
    root.ordering = pbs_ordering

    for agent in agents:
        success = update_plan(root, agent, nodes, nodes_dict, h_func, plotter, middle_plot)
        if not success:
            return None, {}

    root.calc_cost()
    stack = [root]
    iteration = 0
    while len(stack) > 0:
        iteration += 1
        print(f'\n---\nPBS begins iteration: {iteration}\n---\n')
        NEXT_pbs_node = stack.pop()
        there_is_col, c_v, c_e = check_for_collisions(NEXT_pbs_node.plan)
        print(f'collisions: {there_is_col}')
        print(f'ordering of pbs_node {NEXT_pbs_node.index}: {NEXT_pbs_node.ordering}')
        print(f'v_c ({int(len(c_v)/2)}): {c_v}')
        print(f'e_c ({int(len(c_e)/2)}): {c_e}')
        if not there_is_col:
            plotter.plot_mapf_paths(paths_dict=NEXT_pbs_node.plan, nodes=nodes)
            return NEXT_pbs_node.plan, {'PBSNode': NEXT_pbs_node}

        conf, conf_type = choose_conf(c_v, c_e)
        for i in range(2):
            agent = NEXT_pbs_node.agent_dict[conf[i]]
            pbs_node_index += 1
            NEW_pbs_node = PBSNode(agents, agents_dict, pbs_node_index)
            NEW_pbs_node.plan = copy.deepcopy(NEXT_pbs_node.plan)
            NEW_pbs_node.constraints = copy.deepcopy(NEXT_pbs_node.constraints)
            add_new_constraint(NEW_pbs_node, i, conf, conf_type)
            add_new_ordering(NEW_pbs_node, NEXT_pbs_node, i, conf)
            success = update_plan(NEW_pbs_node, agent, nodes, nodes_dict, h_func, plotter, middle_plot)
            if success:
                NEW_pbs_node.calc_cost()
                stack.append(NEW_pbs_node)
                stack.sort(key=lambda x: x.cost, reverse=True)

    # return root.plan
    return None, {}


def create_simple_ordering(n_agents):
    initial_ordering = []
    for i in range(n_agents):
        name_1 = f'agent_{i}'
        for j in range(i+1, n_agents):
            name_2 = f'agent_{j}'
            initial_ordering.append((name_1, name_2))
    return initial_ordering


def main():
    n_agents = 10

    # initial_ordering = create_simple_ordering(n_agents)
    # result = test_mapf_alg_from_pic(algorithm=run_pbs, initial_ordering=initial_ordering, n_agents=n_agents)

    result, info = test_mapf_alg_from_pic(algorithm=run_pbs, initial_ordering=[], n_agents=n_agents)

    there_is_col, vertex_col_list, edge_col_list = check_for_collisions(result)

    print(result)
    # plt.show()
    plt.close()


if __name__ == '__main__':
    main()

