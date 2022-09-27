import copy
import time
import matplotlib.pyplot as plt

from algs.alg_a_star import a_star
from algs.test_mapf_alg import test_mapf_alg_from_pic
from algs.metrics import check_for_collisions, c_v_check_for_agent, c_e_check_for_agent, build_constraints, crossed_time_limit
from algs.topological_sorting import topological_sorting


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
        self.agents = agents
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

    def agent_names(self):
        return [agent.name for agent in self.agents]

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
    agents_names = []
    for a_1, a_2 in pbs_node.ordering_rules:
        agents_names.append(a_1)
        agents_names.append(a_2)
    agents_names = list(set(agents_names))
    # if len(agents_names) > 0:
    #     print(f'\rpbs_node [{pbs_node.index}]: agents_names -> {agents_names}\n')
    pbs_node.partial_order = topological_sorting(nodes=agents_names, sorting_rules=pbs_node.ordering_rules)
    if agent.name in pbs_node.partial_order:
        agent_index = pbs_node.partial_order.index(agent.name)
        higher_order_names = [pbs_node.partial_order[i] for i in range(0, agent_index)]
        lower_order_names = [pbs_node.partial_order[i] for i in range(agent_index + 1, len(pbs_node.partial_order))]
        higher_order_list = [pbs_node.agent_dict[name] for name in higher_order_names]
        lower_order_list = [pbs_node.agent_dict[name] for name in lower_order_names]
        return higher_order_list, lower_order_list
    return [], []


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


def update_path(pbs_node, update_agent, nodes, nodes_dict, h_func, plotter, middle_plot, iter_limit):
    print('\rFUNC: update_path', end='')
    higher_order_list, _ = get_order_lists(pbs_node, update_agent)
    sub_results = {agent.name: pbs_node.plan[agent.name] for agent in higher_order_list}

    c_v_list = c_v_check_for_agent(update_agent.name, pbs_node.plan[update_agent.name], sub_results)
    c_e_list = c_e_check_for_agent(update_agent.name, pbs_node.plan[update_agent.name], sub_results)

    v_constr_dict, e_constr_dict, perm_constr_dict = build_constraints(nodes, sub_results)

    print('\rBEFORE A*', end='')
    new_path = a_star(start=update_agent.start_node, goal=update_agent.goal_node, nodes=nodes,
                      h_func=h_func, v_constr_dict=v_constr_dict, e_constr_dict=e_constr_dict, perm_constr_dict=perm_constr_dict,
                      plotter=plotter, middle_plot=middle_plot, iter_limit=iter_limit)

    if new_path is not None:
        c_v_list_after = c_v_check_for_agent(update_agent.name, new_path, sub_results)
        c_e_list_after = c_e_check_for_agent(update_agent.name, new_path, sub_results)
        if len(c_v_list_after) > 0 or len(c_e_list_after) > 0:
            raise RuntimeError('a_star failed')

    return new_path


# @preprint_func_name
def update_plan(pbs_node, agent, nodes, nodes_dict, h_func, plotter, middle_plot, iter_limit):
    print('\rFUNC: update_plan', end='')
    update_list_names = topological_sorting(nodes=[agent.name], sorting_rules=pbs_node.ordering_rules)
    update_list = [pbs_node.agent_dict[agent_name] for agent_name in update_list_names]
    for update_agent in update_list:
        if collide_check(pbs_node, update_agent) or update_agent.name == agent.name:
            new_path = update_path(pbs_node, update_agent, nodes, nodes_dict, h_func, plotter, middle_plot, iter_limit)
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
    if agent.name == conf[0]:
        h_agent = conf[1]
        l_agent = conf[0]
    elif agent.name == conf[1]:
        h_agent = conf[0]
        l_agent = conf[1]
    else:
        raise RuntimeError('no such name in constr')

    NEW_pbs_node.ordering_rules.append((h_agent, l_agent))

    for (i_1, i_2) in NEW_pbs_node.ordering_rules:
        if (i_2, i_1) in NEW_pbs_node.ordering_rules:
            raise RuntimeError('there are upsidedown orders')

    return agent


def run_pbs(start_nodes, goal_nodes, nodes, nodes_dict, h_func, plotter=None, middle_plot=False, **kwargs):
    start_time = time.time()
    if 'max_time' in kwargs:
        max_time = kwargs['max_time']
    else:
        max_time = 10
    if 'final_plot' in kwargs:
        final_plot = kwargs['final_plot']
    else:
        final_plot = True
    if 'initial_ordering' in kwargs:
        partial_order = kwargs['initial_ordering']
    else:
        partial_order = []
    if 'a_star_iter_limit' in kwargs:
        iter_limit = kwargs['a_star_iter_limit']
    else:
        iter_limit = 1e100
    pbs_node_index = 0
    agents, agents_dict = create_agents(start_nodes, goal_nodes)
    root = PBSNode(agents, agents_dict, pbs_node_index)
    root.partial_order = partial_order
    root.update_ordering_rules()

    for agent in agents:
        success = update_plan(root, agent, nodes, nodes_dict, h_func, plotter, middle_plot, iter_limit)
        if not success:
            return None, {'success_rate': 0}

    root.calc_cost()
    stack = [root]
    iteration = 0
    while len(stack) > 0 and not crossed_time_limit(start_time, max_time):
        iteration += 1
        NEXT_pbs_node = stack.pop()
        there_is_col, c_v, c_e = check_for_collisions(NEXT_pbs_node.plan)
        print(f'\r---\n'
              f'[iter {iteration}][{len(agents)} agents][time: {time.time() - start_time:0.2f}s] '
              f'PBS Node {NEXT_pbs_node.index}, stack: {len(stack)}\n'
              f'partial order: {NEXT_pbs_node.partial_order}\n'
              f'cost: {NEXT_pbs_node.cost}\n'
              f'---\n')

        if not there_is_col:
            if final_plot:
                print(f'order: {topological_sorting(NEXT_pbs_node.agent_names(), NEXT_pbs_node.ordering_rules)}\n')
                print(f'#########################################################')
                print(f'#########################################################')
                print(f'#########################################################')
                plotter.plot_mapf_paths(paths_dict=NEXT_pbs_node.plan, nodes=nodes, plot_per=1)

            runtime = time.time() - start_time
            return NEXT_pbs_node.plan, {
                'PBSNode': NEXT_pbs_node,
                'success_rate': 1, 'sol_quality': NEXT_pbs_node.cost,
                'runtime': runtime, 'iterations_time': runtime}

        conf, conf_type = choose_conf(c_v, c_e)
        for i in range(2):
            pbs_node_index += 1
            NEW_pbs_node = PBSNode(agents, agents_dict, pbs_node_index)
            NEW_pbs_node.plan = copy.deepcopy(NEXT_pbs_node.plan)
            NEW_pbs_node.ordering_rules = copy.deepcopy(NEXT_pbs_node.ordering_rules)

            # NEW_pbs_node.constraints = copy.deepcopy(NEXT_pbs_node.constraints)
            # add_new_constraint(NEW_pbs_node, i, conf, conf_type)

            agent = NEXT_pbs_node.agent_dict[conf[i]]
            agent = add_new_ordering(NEW_pbs_node, NEXT_pbs_node, agent, conf)
            success = update_plan(NEW_pbs_node, agent, nodes, nodes_dict, h_func, plotter, middle_plot, iter_limit)

            if success:
                NEW_pbs_node.calc_cost()
                stack.append(NEW_pbs_node)
                NEW_pbs_node.parent = NEXT_pbs_node
                stack.sort(key=lambda x: x.cost, reverse=True)

    return None, {'success_rate': 0}


def main():
    # initial_ordering = [f'agent_{n}' for n in range(n_agents)]
    # result = test_mapf_alg_from_pic(algorithm=run_pbs, initial_ordering=initial_ordering, n_agents=n_agents)
    for i in range(20):
        print(f'\n[run {i}]')
        result, info = test_mapf_alg_from_pic(algorithm=run_pbs, initial_ordering=[], n_agents=n_agents,
                                              random_seed=random_seed, seed=seed, max_time=5, final_plot=True)

        if not random_seed:
            break

        # plt.show()
        plt.close()


if __name__ == '__main__':
    random_seed = True
    # random_seed = False
    seed = 710
    n_agents = 1

    main()
