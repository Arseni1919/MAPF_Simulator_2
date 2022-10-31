import copy
import math
import random
import cProfile
import pstats
import time
import matplotlib.pyplot as plt
import numpy as np
from simulator_objects import Node
from funcs_plotter.plotter import Plotter
from funcs_graph.nodes_from_pic import make_neighbours, build_graph_from_png
from funcs_graph.map_dimensions import map_dimensions_dict
from funcs_graph.heuristic_funcs import dist_heuristic, h_func_creator, build_heuristic_for_multiple_targets


def get_max_final(perm_constr_dict):
    max_final_time = None
    if perm_constr_dict:
        final_list = [v[0] for k, v in perm_constr_dict.items() if len(v) > 0]
        max_final_time = max(final_list) if len(final_list) > 0 else None
    return max_final_time


def get_node_from_open(open_list):
    # v_list = open_list
    f_dict = {}
    f_vals_list = []
    for node in open_list:
        # TODO: heap list / prioritized list (they use binary search)
        curr_f = node.f()
        f_vals_list.append(curr_f)
        if curr_f not in f_dict:
            f_dict[curr_f] = []
        f_dict[curr_f].append(node)

    smallest_f_nodes = f_dict[min(f_vals_list)]

    h_dict = {}
    h_vals_list = []
    for node in smallest_f_nodes:
        curr_h = node.h
        h_vals_list.append(curr_h)
        if curr_h not in h_dict:
            h_dict[curr_h] = []
        h_dict[curr_h].append(node)

    smallest_h_from_smallest_f_nodes = h_dict[min(h_vals_list)]
    next_node = random.choice(smallest_h_from_smallest_f_nodes)
    return next_node


def get_node(successor_xy_name, node_current, nodes, nodes_dict, open_list, close_list, v_constr_dict, e_constr_dict, perm_constr_dict, max_final_time):
    new_t = node_current.t + 1

    if v_constr_dict:
        if new_t in v_constr_dict[successor_xy_name]:
            return None

    if e_constr_dict:
        if (node_current.x, node_current.y, new_t) in e_constr_dict[successor_xy_name]:
            return None

    if perm_constr_dict:
        if len(perm_constr_dict[successor_xy_name]) > 0:
            if len(perm_constr_dict[successor_xy_name]) != 1:
                raise RuntimeError('len(perm_constr_dict[successor_xy_name]) != 1')
            final_time = perm_constr_dict[successor_xy_name][0]
            if new_t >= final_time:
                return None

    if max_final_time:
        if max_final_time > 0:
            if node_current.t >= max_final_time:
                new_t = max_final_time + 1

    new_ID = f'{successor_xy_name}_{new_t}'

    for open_node in open_list:
        # TODO: dictionary
        if open_node.ID == new_ID:
            return open_node
    for closed_node in close_list:
        if closed_node.ID == new_ID:
            return closed_node

    if nodes_dict:
        node = nodes_dict[successor_xy_name]
        return Node(x=node.x, y=node.y, t=new_t, neighbours=node.neighbours)
    else:
        for node in nodes:
            if node.xy_name == successor_xy_name:
                return Node(x=node.x, y=node.y, t=new_t, neighbours=node.neighbours)
    return None


def reset_nodes(start, goal, nodes):
    _ = [node.reset() for node in nodes]
    return start, goal, nodes


def a_star(start, goal, nodes, h_func,
           v_constr_dict=None, e_constr_dict=None, perm_constr_dict=None,
           plotter=None, middle_plot=False,
           iter_limit=1e100, nodes_dict=None):
    """
    new_t in v_constr_dict[successor_xy_name]
    """
    start_time = time.time()
    # start, goal, nodes = deepcopy_nodes(start, goal, nodes)  # heavy!
    start, goal, nodes = reset_nodes(start, goal, nodes)
    print('\rStarted A*...', end='')
    open_list = []
    close_list = []
    node_current = start
    node_current.h = h_func(node_current, goal)
    open_list.append(node_current)
    max_final_time = get_max_final(perm_constr_dict)
    iteration = 0
    while len(open_list) > 0:
        iteration += 1
        if iteration > iter_limit:
            print(f'\n[ERROR]: out of iterations (more than {iteration})')
            return None, {'runtime': time.time() - start_time, 'n_open': len(open_list), 'n_closed': len(close_list)}
        node_current = get_node_from_open(open_list)  # heavy!
        if node_current.xy_name == goal.xy_name:
            # break
            # if there is a future constraint of a goal
            if len(v_constr_dict[node_current.xy_name]) > 0:
                # we will take the maximum time out of all constraints
                max_t = max(v_constr_dict[node_current.xy_name])
                # and compare to the current time
                # if it is greater, we will continue to expand the search tree
                if node_current.t > max_t:
                    # otherwise break
                    break
                # else:
                #     print('', end='')
            else:
                break
        for successor_xy_name in node_current.neighbours:
            node_successor = get_node(successor_xy_name, node_current, nodes, nodes_dict, open_list, close_list, v_constr_dict, e_constr_dict, perm_constr_dict, max_final_time)  # heavy!
            successor_current_time = node_current.t + 1  # h(now, next)
            if node_successor is None:
                continue
            if node_successor in open_list:
                if node_successor.t <= successor_current_time:
                    continue
            elif node_successor in close_list:
                if node_successor.t <= successor_current_time:
                    continue
                close_list.remove(node_successor)
                open_list.append(node_successor)
            else:
                open_list.append(node_successor)
                node_successor.h = h_func(node_successor, goal)
            node_successor.t = successor_current_time
            node_successor.g = node_successor.t
            node_successor.parent = node_current

        open_list.remove(node_current)
        close_list.append(node_current)

        if plotter and middle_plot and iteration % 10 == 0:
            plotter.plot_lists(open_list=open_list, closed_list=close_list, start=start, goal=goal, nodes=nodes)
        print(f'\r(a_star) iter: {iteration}, open: {len(open_list)}', end='')

    path = None
    if node_current.xy_name == goal.xy_name:
        path = []
        while node_current is not None:
            path.append(node_current)
            node_current = node_current.parent
        path.reverse()

    if plotter and middle_plot:
        plotter.plot_lists(open_list=open_list, closed_list=close_list, start=start, goal=goal, path=path, nodes=nodes)
    # print('\rFinished A*.', end='')
    if path is None:
        print()
    return path, {'runtime': time.time() - start_time, 'n_open': len(open_list), 'n_closed': len(close_list)}


def main():
    nodes = [
        Node(x=1, y=1, neighbours=[]),
        Node(x=1, y=2, neighbours=[]),
        Node(x=1, y=3, neighbours=[]),
        Node(x=1, y=4, neighbours=[]),
        Node(x=2, y=1, neighbours=[]),
        Node(x=2, y=2, neighbours=[]),
        Node(x=2, y=3, neighbours=[]),
        Node(x=2, y=4, neighbours=[]),
        Node(x=3, y=1, neighbours=[]),
        Node(x=3, y=2, neighbours=[]),
        Node(x=3, y=3, neighbours=[]),
        Node(x=3, y=4, neighbours=[]),
        Node(x=4, y=1, neighbours=[]),
        Node(x=4, y=2, neighbours=[]),
        Node(x=4, y=3, neighbours=[]),
        Node(x=4, y=4, neighbours=[]),
    ]
    make_neighbours(nodes)
    node_start = nodes[0]
    node_goal = nodes[-1]
    plotter = Plotter(map_dim=(5, 5))
    result = a_star(start=node_start, goal=node_goal, nodes=nodes, h_func=dist_heuristic, plotter=plotter, middle_plot=True)

    plt.show()
    print(result)
    plt.close()


def try_a_map_from_pic():
    # img_png = 'lak109d.png'
    # img_png = '19_20_warehouse.png'
    # img_png = 'den101d.png'
    # img_png = 'rmtst.png'
    # img_png = 'lak505d.png'
    # img_png = 'lak503d.png'
    # img_png = 'ost003d.png'
    # img_png = 'brc202d.png'
    # img_png = 'den520d.png'
    img_png = 'warehouse-10-20-10-2-1.png'
    map_dim = map_dimensions_dict[img_png]
    nodes, nodes_dict = build_graph_from_png(img_png=img_png, path='../maps', show_map=False)
    # ------------------------- #
    # x_start, y_start = 97, 99
    # x_goal, y_goal = 38, 89
    # node_start = [node for node in nodes if node.x == x_start and node.y == y_start][0]
    # node_goal = [node for node in nodes if node.x == x_goal and node.y == y_goal][0]
    # ------------------------- #
    # node_start = nodes[100]
    # node_goal = nodes[-1]
    # ------------------------- #
    node_start = random.choice(nodes)
    node_goal = random.choice(nodes)
    print(f'start: {node_start.x}, {node_start.y} -> goal: {node_goal.x}, {node_goal.y}')
    # ------------------------- #
    # ------------------------- #
    plotter = Plotter(map_dim=map_dim)
    # plotter = None
    # ------------------------- #
    # ------------------------- #
    h_dict = build_heuristic_for_multiple_targets([node_goal], nodes, map_dim, plotter=plotter, middle_plot=False)
    h_func = h_func_creator(h_dict)
    # ------------------------- #
    # h_func = dist_heuristic
    # ------------------------- #
    # ------------------------- #
    # constraint_dict = None
    v_constr_dict = {node.xy_name: [] for node in nodes}
    # v_constr_dict = {'30_12': [69], '29_12': [68, 69]}
    perm_constr_dict = {node.xy_name: [] for node in nodes}
    perm_constr_dict['74_9'].append(10)
    # ------------------------- #
    # ------------------------- #
    # result = a_star(start=node_start, goal=node_goal, nodes=nodes, h_func=h_func, plotter=plotter, middle_plot=False)
    profiler.enable()
    result = a_star(start=node_start, goal=node_goal, nodes=nodes, h_func=h_func,
                    v_constr_dict=v_constr_dict, perm_constr_dict=perm_constr_dict,
                    plotter=plotter, middle_plot=True, nodes_dict=nodes_dict)
    profiler.disable()
    print('The result is:', *[node.xy_name for node in result], sep='->')
    print('The result is:', *[node.ID for node in result], sep='->')
    # ------------------------- #
    # ------------------------- #
    plt.show()
    # plt.close()


if __name__ == '__main__':
    # random_seed = True
    random_seed = False
    seed = random.choice(range(1000)) if random_seed else 121
    random.seed(seed)
    np.random.seed(seed)
    print(f'SEED: {seed}')
    # main()
    profiler = cProfile.Profile()
    # profiler.enable()
    try_a_map_from_pic()
    # profiler.disable()
    # stats.print_stats()
    stats = pstats.Stats(profiler).sort_stats('cumtime')
    stats.dump_stats('../stats/results_a_star.pstat')


# def deepcopy_nodes(start, goal, nodes):
#     copy_nodes_dict = {node.xy_name: copy.deepcopy(node) for node in nodes}
#     copy_start = copy_nodes_dict[start.xy_name]
#     copy_goal = copy_nodes_dict[goal.xy_name]
#     copy_nodes = list(copy_nodes_dict.values())
#     return copy_start, copy_goal, copy_nodes
