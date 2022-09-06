from alg_a_star import a_star
from test_mapf_alg import test_mapf_alg_from_pic


class PBSAgent:
    def __init__(self, index, start_node, goal_node):
        self.index = index
        self.name = f'agent_{index}'
        self.start_node = start_node
        self.start_xy = self.start_node.xy_name
        self.goal_node = goal_node
        self.goal_xy = self.goal_node.xy_name
        self.order = 0


class PBSNode:
    def __init__(self, agents, agents_dict):
        self.agent = agents
        self.agent_dict = agents_dict
        self.plan = {agent.name: [] for agent in agents}
        self.cost = -1
        self.constraints = {}
        self.ordering = []


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
    _, lower_order_list = get_order_lists(pbs_node, agent)

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


def vertex_collision_check(pbs_node, update_agent, higher_order_list):
    vertex_col_list = []
    for h_agent in higher_order_list:
        path = pbs_node.plan[h_agent.name]
        vertex_list = [(node.x, node.y, node.t) for node in path]
        vertex_col_list.extend(vertex_list)
    agent_path = pbs_node.plan[update_agent.name]
    for node in agent_path:
        if (node.x, node.y, node.t) in vertex_col_list:
            return True
    return False


def edge_collision_check(pbs_node, update_agent, higher_order_list):
    edge_col_list = []
    for h_agent in higher_order_list:
        path = pbs_node.plan[h_agent.name]
        edge_list = []
        if len(path) > 1:
            prev_node = path[0]
            for node in path[1:]:
                edge = (prev_node.x, prev_node.y, node.x, node.y, node.t)
                edge_list.append(edge)
        edge_col_list.extend(edge_list)

    agent_path = pbs_node.plan[update_agent.name]
    if len(agent_path) > 1:
        prev_node = agent_path[0]
        for node in agent_path[1:]:
            edge = (prev_node.x, prev_node.y, node.x, node.y, node.t)
            if edge in edge_col_list:
                return True
    return False


def collide_check(pbs_node, update_agent):
    higher_order_list, _ = get_order_lists(pbs_node, update_agent)

    if vertex_collision_check(pbs_node, update_agent, higher_order_list):
        return True

    if edge_collision_check(pbs_node, update_agent, higher_order_list):
        return True

    return False


def update_path(pbs_node, update_agent, nodes, nodes_dict, h_func, plotter, middle_plot):
    higher_order_list, _ = get_order_lists(pbs_node, update_agent)
    longest_path = max([len(path) for path in pbs_node.plan.values()])
    constraint_dict = {node.xy_name: [] for node in nodes}
    for i_agent in higher_order_list:
        last_time = 0
        i_path = pbs_node.plan[i_agent.name]
        for node in i_path:

            # vertex conf
            constraint_dict[node.xy_name].append(node.t)
            last_time = node.t

            # edge conf
            if node.t > 0:
                constraint_dict[node.xy_name].append(node.t - 1)

        for t in range(last_time, longest_path):
            constraint_dict[i_path[-1].xy_name].append(t)

    new_path = a_star(start=update_agent.start_node, goal=update_agent.goal_node, nodes=nodes,
                      h_func=h_func, constraint_dict=constraint_dict,
                      plotter=plotter, middle_plot=middle_plot)
    return new_path


def update_plan(pbs_node, agent, nodes, nodes_dict, h_func, plotter, middle_plot):
    update_list = topological_sorting(pbs_node, agent)

    for update_agent in update_list:
        if collide_check(pbs_node, update_agent) or update_agent.name == agent.name:
            new_path = update_path(pbs_node, update_agent, nodes, nodes_dict, h_func, plotter, middle_plot)
            if new_path is None:
                return False
            pbs_node.plan[update_agent.name] = new_path
    return True


def run_pbs(start_nodes, goal_nodes, nodes, nodes_dict, h_func, plotter=None, middle_plot=False, **kwargs):
    pbs_ordering = kwargs['initial_ordering']
    agents, agents_dict = create_agents(start_nodes, goal_nodes)
    root = PBSNode(agents, agents_dict)
    root.ordering = pbs_ordering

    for agent in agents:
        success = update_plan(root, agent, nodes, nodes_dict, h_func, plotter, middle_plot)
        if not success:
            return None

    plotter.plot_mapf_paths(paths_dict=root.plan, nodes=nodes)

    return root.plan


def create_simple_ordering(n_agents):
    initial_ordering = []
    for i in range(n_agents):
        name_1 = f'agent_{i}'
        for j in range(i+1, n_agents):
            name_2 = f'agent_{j}'
            initial_ordering.append((name_1, name_2))
    return initial_ordering


def main():
    n_agents = 50

    initial_ordering = create_simple_ordering(n_agents)
    result = test_mapf_alg_from_pic(algorithm=run_pbs, initial_ordering=initial_ordering, n_agents=n_agents)

    # result = test_mapf_alg_from_pic(algorithm=run_pbs, initial_ordering=[], n_agents=n_agents)
    print(result)


if __name__ == '__main__':
    main()

