import matplotlib.pyplot as plt

from alg_a_star import a_star
from test_mapf_alg import test_mapf_alg_from_pic
from metrics import check_for_collisions, c_v_check_for_agent, c_e_check_for_agent


class DSAgent:
    def __init__(self):
        pass


def run_ds_mapf(start_nodes, goal_nodes, nodes, nodes_dict, h_func, plotter=None, middle_plot=False, **kwargs):
    return None, {}


def main():
    result, info = test_mapf_alg_from_pic(algorithm=run_ds_mapf, n_agents=n_agents, random_seed=random_seed, seed=seed)
    print(result)
    # plt.show()
    plt.close()


if __name__ == '__main__':
    # random_seed = True
    random_seed = False
    seed = 250
    n_agents = 15

    main()


