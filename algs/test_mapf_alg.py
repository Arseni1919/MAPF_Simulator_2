import random
import matplotlib.pyplot as plt
import numpy as np

from funcs_plotter.plotter import Plotter
from funcs_graph.map_dimensions import map_dimensions_dict
from funcs_graph.nodes_from_pic import build_graph_from_png
from funcs_graph.heuristic_funcs import dist_heuristic, h_func_creator, build_heuristic_for_multiple_targets
from funcs_graph.heuristic_funcs import parallel_build_heuristic_for_multiple_targets


def test_mapf_alg_from_pic(algorithm, **kwargs):
    random_seed = kwargs['random_seed']
    seed = kwargs['seed']
    if random_seed:
        seed = random.choice(range(1000))
    print(f'SEED: {seed}')

    random.seed(seed)
    np.random.seed(seed)

    # img_png = 'lak109d.png'
    # img_png = '19_20_warehouse.png'
    img_png = 'warehouse-10-20-10-2-1.png'
    # img_png = 'warehouse-10-20-10-2-2.png'
    # img_png = 'ht_chantry.png'
    # img_png = 'Berlin_1_256.png'
    # img_png = 'den101d.png'
    # img_png = 'rmtst.png'
    # img_png = 'lak505d.png'
    # img_png = 'lak503d.png'
    # img_png = 'ost003d.png'
    # img_png = 'brc202d.png'
    # img_png = 'den520d.png'
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
    n_agents = kwargs['n_agents']
    sample_nodes = random.sample(nodes, 2 * n_agents)
    start_nodes = sample_nodes[:n_agents]
    goal_nodes = sample_nodes[n_agents:]
    # ------------------------- #
    # ------------------------- #
    plotter = Plotter(map_dim=map_dim)
    # ------------------------- #
    # ------------------------- #
    # h_dict = build_heuristic_for_multiple_targets(goal_nodes, nodes, map_dim, plotter=plotter, middle_plot=False)
    h_dict = parallel_build_heuristic_for_multiple_targets(goal_nodes, nodes, map_dim, plotter=plotter, middle_plot=False)
    h_func = h_func_creator(h_dict)

    # h_func = dist_heuristic

    # ------------------------- #
    # ------------------------- #
    result = algorithm(start_nodes=start_nodes, goal_nodes=goal_nodes, nodes=nodes, nodes_dict=nodes_dict, h_func=h_func,
                       plotter=plotter, middle_plot=False, **kwargs)

    print(f'The result: {result}')
    # ------------------------- #
    # ------------------------- #
    # plt.show()
    # plt.close()
    return result


def main():
    pass


if __name__ == '__main__':
    main()
