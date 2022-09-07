import matplotlib.pyplot as plt

from alg_a_star import a_star
from test_mapf_alg import test_mapf_alg_from_pic
from metrics import check_for_collisions, c_v_check_for_agent, c_e_check_for_agent


class DISTOAgent:


def run_disto_mapf(start_nodes, goal_nodes, nodes, nodes_dict, h_func, plotter=None, middle_plot=False, **kwargs):
    pass


def main():
    n_agents = 10
    result, info = test_mapf_alg_from_pic(algorithm=run_disto_mapf, n_agents=n_agents)

    print(result)
    # plt.show()
    plt.close()


if __name__ == '__main__':
    main()


