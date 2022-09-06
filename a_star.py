import math
import random

import matplotlib.pyplot as plt
import numpy as np
from simulator_objects import Node
from funcs_plotter.plotter import Plotter
from funcs_graph.nodes_from_pic import make_neighbours, build_graph_from_png
from funcs_graph.map_dimensions import map_dimensions_dict
from funcs_graph.heuristic_funcs import build_heuristic_for_one_target, build_heuristic_for_multiple_targets


def dist_heuristic(from_node, to_node):
    return np.abs(from_node.x - to_node.x) + np.abs(from_node.y - to_node.y)
    # return np.sqrt((from_node.x - to_node.x) ** 2 + (from_node.y - to_node.y) ** 2)


def h_func_creator(h_dict):
    def h_func(from_node, to_node):
        if to_node.xy_name in h_dict:
            h_value = h_dict[to_node.xy_name][from_node.x, from_node.y]
            if h_value > 0:
                return h_value
        return np.abs(from_node.x - to_node.x) + np.abs(from_node.y - to_node.y)
        # return np.sqrt((from_node.x - to_node.x) ** 2 + (from_node.y - to_node.y) ** 2)
    return h_func


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


def get_node(successor_xy_name, node_current, nodes, open_list, close_list, constraint_dict):
    if constraint_dict:
        new_t = node_current.t + 1
        if successor_xy_name in constraint_dict and new_t in constraint_dict[successor_xy_name]:
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


def a_star(start, goal, nodes, h_func, constraint_dict=None, plotter=None, middle_plot=False):
    print('Started A*...')
    open_list = []
    close_list = []
    node_current = start
    node_current.h = h_func(node_current, goal)
    open_list.append(node_current)
    iteration = 0
    while len(open_list) > 0:
        iteration += 1
        node_current = get_node_from_open(open_list)
        if node_current.xy_name == goal.xy_name:
            break
        for successor_xy_name in node_current.neighbours:
            node_successor = get_node(successor_xy_name, node_current, nodes, open_list, close_list, constraint_dict)
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
        print(f'\riter: {iteration}', end='')

    path = None
    if node_current.xy_name == goal.xy_name:
        path = []
        while node_current is not None:
            path.append(node_current)
            node_current = node_current.parent
        path.reverse()

    if plotter:
        plotter.plot_lists(open_list=open_list, closed_list=close_list, start=start, goal=goal, path=path, nodes=nodes)
    print('Finished A*.')
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
    # img_png = 'den101d.png'
    # img_png = 'rmtst.png'
    # img_png = 'lak505d.png'
    img_png = 'lak503d.png'
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
    constraint_dict = {'30_12': [69], '29_12': [68, 69]}
    # ------------------------- #
    # ------------------------- #
    # result = a_star(start=node_start, goal=node_goal, nodes=nodes, h_func=h_func, plotter=plotter, middle_plot=False)
    result = a_star(start=node_start, goal=node_goal, nodes=nodes, h_func=h_func, constraint_dict=constraint_dict, plotter=plotter, middle_plot=True)
    print('The result is:', *[node.xy_name for node in result], sep='->')
    print('The result is:', *[node.ID for node in result], sep='->')
    # ------------------------- #
    # ------------------------- #
    plt.show()
    plt.close()


if __name__ == '__main__':
    # main()
    try_a_map_from_pic()
