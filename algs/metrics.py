

def c_v_check_for_agent(agent_1: str, path_1: list, results):
    """
    c_v_for_agent_list: (agent name 1, agent name 2, x, y, t)
    """
    if type(agent_1) is not str:
        raise RuntimeError('type(agent_1) is not str')
    c_v_for_agent_list = []
    if len(path_1) < 1:
        return c_v_for_agent_list
    for agent_2, path_2 in results.items():
        if type(agent_2) is not str:
            raise RuntimeError('type(agent_2) is not str')
        if agent_2 != agent_1:
            for t in range(max(len(path_1), len(path_2))):
                node_1 = path_1[min(t, len(path_1) - 1)]
                node_2 = path_2[min(t, len(path_2) - 1)]
                if (node_1.x, node_1.y) == (node_2.x, node_2.y):
                    c_v_for_agent_list.append((agent_1, agent_2, node_1.x, node_1.y, t))
    return c_v_for_agent_list


def vertex_col_check(results):
    vertex_col_list = []
    for agent_1, path_1 in results.items():
        c_v_for_agent_list = c_v_check_for_agent(agent_1, path_1, results)
        vertex_col_list.extend(c_v_for_agent_list)
    return vertex_col_list


def c_e_check_for_agent(agent_1: str, path_1, results):
    """
    c_e_check_for_agent: (agent name 1, agent name 2, x, y, x, y, t)
    """
    if type(agent_1) is not str:
        raise RuntimeError('type(agent_1) is not str')
    c_e_for_agent_list = []
    if len(path_1) <= 1:
        return c_e_for_agent_list
    for agent_2, path_2 in results.items():
        if agent_2 != agent_1:
            if len(path_2) > 1:
                prev_node_1 = path_1[0]
                prev_node_2 = path_2[0]
                for t in range(max(len(path_1), len(path_2))):
                    node_1 = path_1[min(t, len(path_1) - 1)]
                    node_2 = path_2[min(t, len(path_2) - 1)]
                    if (prev_node_1.x, prev_node_1.y, node_1.x, node_1.y) == (prev_node_2.x, prev_node_2.y, node_2.x, node_2.y):
                        c_e_for_agent_list.append((agent_1, agent_2, prev_node_1.x, prev_node_1.y, node_1.x, node_1.y, t))
                    prev_node_1 = node_1
                    prev_node_2 = node_2
    return c_e_for_agent_list


def edge_col_check(results):
    edge_col_list = []
    for agent_1, path_1 in results.items():
        e_v_for_agent_list = c_e_check_for_agent(agent_1, path_1, results)
        edge_col_list.extend(e_v_for_agent_list)
    return edge_col_list


def check_for_collisions(results):
    """
    results: {'agent str': [Nodes list]}
    """
    if results:
        vertex_col_list = vertex_col_check(results)
        edge_col_list = edge_col_check(results)
        there_is_col = len(vertex_col_list) > 0 or len(edge_col_list) > 0
        return there_is_col, vertex_col_list, edge_col_list
    return True, [], []


def main():
    pass


if __name__ == '__main__':
    main()


