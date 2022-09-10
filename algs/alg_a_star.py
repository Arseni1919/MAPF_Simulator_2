import copy
import math
import random

import matplotlib.pyplot as plt
import numpy as np
from simulator_objects import Node
from funcs_plotter.plotter import Plotter
from funcs_graph.nodes_from_pic import make_neighbours, build_graph_from_png
from funcs_graph.map_dimensions import map_dimensions_dict
from funcs_graph.heuristic_funcs import dist_heuristic, h_func_creator, build_heuristic_for_multiple_targets


def get_node_from_open(open_list):
    v_list = open_list
    t_val = min([node.f() for node in v_list])
    out_nodes = [node for node in v_list if node.f() == t_val]
    v_list = out_nodes
    some_v = min([node.h for node in v_list])
    out_nodes = [node for node in v_list if node.h == some_v]
    # print([node.ID for node in t_nodes])
    next_node = random.choice(out_nodes)
    return next_node


def get_node(successor_xy_name, node_current, nodes, open_list, close_list, v_constr_dict, e_constr_dict, perm_constr_dict):
    new_t = node_current.t + 1
    if v_constr_dict:
        if successor_xy_name in v_constr_dict and new_t in v_constr_dict[successor_xy_name]:
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
    new_ID = f'{successor_xy_name}_{node_current.t + 1}'
    for node in nodes:
        if node.xy_name == successor_xy_name:
            for open_node in open_list:
                if open_node.ID == new_ID:
                    return open_node
            for closed_node in close_list:
                if closed_node.ID == new_ID:
                    return closed_node
            return Node(x=node.x, y=node.y, t=node_current.t + 1, neighbours=node.neighbours)
    return None


def deepcopy_nodes(start, goal, nodes):
    copy_nodes_dict = {node.xy_name: copy.deepcopy(node) for node in nodes}
    copy_start = copy_nodes_dict[start.xy_name]
    copy_goal = copy_nodes_dict[goal.xy_name]
    copy_nodes = list(copy_nodes_dict.values())
    return copy_start, copy_goal, copy_nodes


def a_star(start, goal, nodes, h_func, v_constr_dict=None, e_constr_dict=None, perm_constr_dict=None, plotter=None, middle_plot=False):
    """
    new_t in v_constr_dict[successor_xy_name]
    """
    start, goal, nodes = deepcopy_nodes(start, goal, nodes)
    print('\rStarted A*...', end='')
    open_list = []
    close_list = []
    node_current = start
    node_current.h = h_func(node_current, goal)
    open_list.append(node_current)
    iteration = 0
    while len(open_list) > 0:
        iteration += 1
        if iteration > 1e3:
            print('[ERROR]: out of iterations')
            return None
        node_current = get_node_from_open(open_list)
        if node_current.xy_name == goal.xy_name:
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
            node_successor = get_node(successor_xy_name, node_current, nodes, open_list, close_list, v_constr_dict, e_constr_dict, perm_constr_dict)
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
    return path


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
    img_png = 'den101d.png'
    # img_png = 'rmtst.png'
    # img_png = 'lak505d.png'
    # img_png = 'lak503d.png'
    # img_png = 'ost003d.png'
    # img_png = 'brc202d.png'
    # img_png = 'den520d.png'
    map_dim = map_dimensions_dict[img_png]
    nodes, nodes_dict = build_graph_from_png(img_png=img_png, path='maps', show_map=False)
    # ------------------------- #
    # x_start, y_start = 97, 99
    # x_goal, y_goal = 38, 89
    # node_start = [node for node in nodes if node.x == x_start and node.y == y_start][0]
    # node_goal = [node for node in nodes if node.x == x_goal and node.y == y_goal][0]
    # ------------------------- #
    node_start = nodes[100]
    node_goal = nodes[-1]
    # ------------------------- #
    # node_start = random.choice(nodes)
    # node_goal = random.choice(nodes)
    print(f'start: {node_start.x}, {node_start.y} -> goal: {node_goal.x}, {node_goal.y}')
    # ------------------------- #
    # ------------------------- #
    plotter = Plotter(map_dim=map_dim)
    # ------------------------- #
    # ------------------------- #
    h_dict = build_heuristic_for_multiple_targets([node_goal], nodes, map_dim, plotter=plotter, middle_plot=False)
    h_func = h_func_creator(h_dict)
    # ------------------------- #
    # h_func = dist_heuristic
    # ------------------------- #
    # ------------------------- #
    constraint_dict = None
    # v_constr_dict = {'30_12': [69], '29_12': [68, 69]}
    # ------------------------- #
    # ------------------------- #
    # result = a_star(start=node_start, goal=node_goal, nodes=nodes, h_func=h_func, plotter=plotter, middle_plot=False)
    result = a_star(start=node_start, goal=node_goal, nodes=nodes, h_func=h_func, v_constr_dict=constraint_dict, plotter=plotter, middle_plot=True)
    print('The result is:', *[node.xy_name for node in result], sep='->')
    print('The result is:', *[node.ID for node in result], sep='->')
    # ------------------------- #
    # ------------------------- #
    plt.show()
    plt.close()


if __name__ == '__main__':
    # main()
    try_a_map_from_pic()
