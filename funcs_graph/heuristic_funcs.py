import copy
import random
import numpy as np
import matplotlib.pyplot as plt

import asyncio

from simulator_objects import Node
from funcs_graph.map_dimensions import map_dimensions_dict
from funcs_graph.nodes_from_pic import build_graph_from_png
from funcs_plotter.plotter import Plotter


def get_node_from_open(open_list):
    v_list = open_list
    some_v = min([node.g for node in v_list])
    out_nodes = [node for node in v_list if node.g == some_v]
    # print([node.ID for node in t_nodes])
    next_node = random.choice(out_nodes)
    return next_node


def get_node(successor_xy_name, node_current, nodes, open_list, close_list):
    for node in nodes:
        if node.xy_name == successor_xy_name and node_current.xy_name != successor_xy_name:
            return node
    return None


# for open_node in open_list:
#     if open_node.xy_name == node.xy_name:
#         return open_node
# for closed_node in close_list:
#     if closed_node.xy_name == node.xy_name:
#         return closed_node


def build_heuristic_for_multiple_targets(target_nodes, nodes, map_dim, to_save=True, plotter=None, middle_plot=False):
    h_dict = {}
    for node in target_nodes:
        h_table = build_heuristic_for_one_target(node, nodes, map_dim, to_save, plotter, middle_plot)
        h_dict[node.xy_name] = h_table

    return h_dict


def build_heuristic_for_one_target(target_node, nodes, map_dim, to_save=True, plotter=None, middle_plot=False):
    print('Started to build heuristic...')
    copy_nodes = copy.deepcopy(nodes)
    target_node = [node for node in copy_nodes if node.xy_name == target_node.xy_name][0]
    open_list = []
    close_list = []
    open_list.append(target_node)
    iteration = 0
    while len(open_list) > 0:
        iteration += 1
        node_current = get_node_from_open(open_list)
        # if node_current.xy_name == '30_12':
        #     print()
        for successor_xy_name in node_current.neighbours:
            node_successor = get_node(successor_xy_name, node_current, copy_nodes, open_list, close_list)
            if node_successor:
                successor_current_g = node_current.g + 1  # h(now, next)
                if node_successor in open_list:
                    if node_successor.g <= successor_current_g:
                        continue
                elif node_successor in close_list:
                    if node_successor.g <= successor_current_g:
                        continue
                    close_list.remove(node_successor)
                    open_list.append(node_successor)
                else:
                    open_list.append(node_successor)
                node_successor.g = successor_current_g
                # if node_successor.xy_name == '31_12':
                #     print()
                node_successor.parent = node_current

        open_list.remove(node_current)
        close_list.append(node_current)

        if plotter and middle_plot and iteration % 100 == 0:
            plotter.plot_lists(open_list=open_list, closed_list=close_list, start=target_node, nodes=copy_nodes)
        if iteration % 100 == 0:
            print(f'\riter: {iteration}', end='')

    if plotter:
        plotter.plot_lists(open_list=open_list, closed_list=close_list, start=target_node, nodes=copy_nodes)

    h_table = np.zeros(map_dim)
    for node in copy_nodes:
        h_table[node.x, node.y] = node.g
    # h_dict = {target_node.xy_name: h_table}
    print('\nFinished to build heuristic.')
    return h_table


def main():
    img_png = 'den520d.png'
    map_dim = map_dimensions_dict[img_png]
    nodes, nodes_dict = build_graph_from_png(img_png=img_png, path='../maps', show_map=False)
    x_goal, y_goal = 38, 89
    node_goal = [node for node in nodes if node.x == x_goal and node.y == y_goal][0]
    plotter = Plotter(map_dim=map_dim)
    h_table = build_heuristic_for_one_target(node_goal, nodes, map_dim, plotter=plotter, middle_plot=True)
    # h_table = build_heuristic_to_one_target(node_goal, nodes, map_dim, plotter=plotter, middle_plot=False)
    print(h_table)
    plt.show()
    plt.close()


if __name__ == '__main__':
    main()
