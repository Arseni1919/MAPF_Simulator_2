import time


def limit_is_crossed(runtime, alg_info, **kwargs):
    if 'limit_type' not in kwargs:
        raise RuntimeError('limit_type not in kwargs')

    limit_type = kwargs['limit_type']
    max_time = kwargs['max_time'] if 'max_time' in kwargs else 60
    a_star_calls_limit = kwargs['a_star_calls_limit'] if 'a_star_calls_limit' in kwargs else 1e100
    a_star_closed_nodes_limit = kwargs['a_star_closed_nodes_limit'] if 'a_star_closed_nodes_limit' in kwargs else 1e100

    # PRINT
    a_star_n_closed_counter = sum(alg_info['a_star_n_closed'])
    print(f'\n{runtime =}'
          f'\n{alg_info["dist_runtime"] = }'
          f'\n{alg_info["a_star_calls_counter"] = }'
          f'\n{alg_info["a_star_calls_counter_dist"] = }'
          f'\n{a_star_n_closed_counter = }'
          f'\n{alg_info["a_star_n_closed_dist"] = }')

    if limit_type == 'norm_time':
        return runtime > max_time * 60
    elif limit_type == 'dist_time':
        return alg_info['dist_runtime'] > max_time * 60
    elif limit_type == 'norm_a_star_calls':
        return alg_info['a_star_calls_counter'] >= a_star_calls_limit
    elif limit_type == 'dist_a_star_calls':
        return alg_info['a_star_calls_counter_dist'] >= a_star_calls_limit
    elif limit_type == 'norm_a_star_closed':
        a_star_n_closed_counter = sum(alg_info['a_star_n_closed'])
        return a_star_n_closed_counter >= a_star_closed_nodes_limit
    elif limit_type == 'dist_a_star_closed':
        return alg_info['a_star_n_closed_dist'] >= a_star_closed_nodes_limit
    else:
        raise RuntimeError('no valid limit_type')


def crossed_time_limit(start_time, max_time_minutes):
    elapsed = time.time() - start_time
    return elapsed > max_time_minutes * 60


def get_alg_info_dict(**kwargs):
    alg_info = {'success_rate': 0,
                'sol_quality': 0,
                'runtime': 0,
                'a_star_calls_counter': 0,
                'a_star_calls_counter_dist': 0,
                'dist_runtime': 0,
                'a_star_n_closed_dist': 0,
                'a_star_runtimes': [],
                'a_star_n_closed': [],
                'n_agents_conf': []}
    alg_info.update(kwargs)
    return alg_info


def get_agents_in_conf(c_v_list, c_e_list):
    agents_in_conf = [conf[1] for conf in c_v_list]
    agents_in_conf.extend([conf[1] for conf in c_e_list])
    agents_in_conf = list(set(agents_in_conf))
    return agents_in_conf


def build_constraints(nodes, other_paths):
    v_constr_dict = {node.xy_name: [] for node in nodes}
    e_constr_dict = {node.xy_name: [] for node in nodes}
    perm_constr_dict = {node.xy_name: [] for node in nodes}

    for agent_name, path in other_paths.items():
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
    return v_constr_dict, e_constr_dict, perm_constr_dict


def c_v_check_for_agent(agent_1: str, path_1, results):
    """
    c_v_for_agent_list: (agents name 1, agents name 2, x, y, t)
    """
    if type(agent_1) is not str:
        raise RuntimeError('type(agent_1) is not str')
    c_v_for_agent_list = []
    if len(path_1) < 1:
        return c_v_for_agent_list
    for agent_2, path_2 in results.items():
        # if type(agent_2) is not str:
        #     raise RuntimeError('type(agent_2) is not str')
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
    c_e_check_for_agent: (agents name 1, agents name 2, x, y, x, y, t)
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
                for t in range(1, min(len(path_1), len(path_2))):
                    node_1 = path_1[t]
                    node_2 = path_2[t]
                    if (prev_node_1.x, prev_node_1.y, node_1.x, node_1.y) == (node_2.x, node_2.y, prev_node_2.x, prev_node_2.y):
                        c_e_for_agent_list.append((agent_1, agent_2, prev_node_1.x, prev_node_1.y, node_1.x, node_1.y, t))
                    prev_node_1 = node_1
                    prev_node_2 = node_2
    return c_e_for_agent_list


def edge_col_check(results):
    edge_col_list = []
    for agent_1, path_1 in results.items():
        c_e_for_agent_list = c_e_check_for_agent(agent_1, path_1, results)
        edge_col_list.extend(c_e_for_agent_list)
    return edge_col_list


def check_for_collisions(results):
    """
    results: {'agents str': [Nodes heap_list]}
    """
    print('\nStart check_for_collisions...')
    if results:
        vertex_col_list = vertex_col_check(results)
        edge_col_list = edge_col_check(results)
        there_is_col = len(vertex_col_list) > 0 or len(edge_col_list) > 0
        return there_is_col, vertex_col_list, edge_col_list
    return True, [], []


def check_plan(agents, plan, alg_name, alg_info, runtime, iteration):
    plan_lngths = [len(path) for path in plan.values()]
    if 0 in plan_lngths:
        raise RuntimeError('0 in plan_lngths')
    cost = sum([len(path) for path in plan.values()])
    print(f'\r---\n'
          f'[{alg_name}][{len(agents)} agents][A* calls: {alg_info["a_star_calls_counter"]}][time: {runtime:0.2f}s][iter {iteration}][A* dist calls: ]\n'
          f'cost: {cost}\n'
          f'---\n')

    there_is_col, c_v, c_e = check_for_collisions(plan)
    return there_is_col, c_v, c_e, cost


def main():
    pass


if __name__ == '__main__':
    main()


