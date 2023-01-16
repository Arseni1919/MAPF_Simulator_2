import json
from datetime import datetime

import matplotlib.pyplot as plt

from funcs_plotter.plotter import Plotter
from algs.alg_DS_MAPF import run_ds_mapf
from algs.alg_PBS import run_pbs


def show_results(file_dir, plotter):
    """
    to_save_dict = {
        'statistics_dict': statistics_dict,
        'runs_per_n_agents': runs_per_n_agents,
        'n_agents_list': n_agents_list,
        'algs_to_test_names': heap_list(algs_to_test_dict.keys()),
        'img_dir': img_dir

    }
    """
    with open(f'{file_dir}', 'r') as openfile:
        # Reading from json file
        json_object = json.load(openfile)
    statistics_dict = json_object['stats_dict']
    runs_per_n_agents = json_object['runs_per_n_agents']
    n_agents_list = json_object['n_agents_list']
    img_png = json_object['img_dir']
    # algs_to_test_names = json_object['algs_to_test_names']
    algs_to_test_dict = json_object['algs_to_test_dict']
    plotter.plot_big_test(statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, img_png, is_json=True)
    plt.show()


def main():

    """ 20 RUNS PER N AGENTS: """
    # file_dir = f'../logs_for_graphs/2023-01-06--18-28_ALGS-3_RUNS-20_MAP-empty-48-48.json'
    # file_dir = f'../logs_for_graphs/2023-01-06--20-44_ALGS-3_RUNS-20_MAP-random-64-64-10.json'
    # file_dir = f'../logs_for_graphs/2023-01-09--08-23_ALGS-3_RUNS-20_MAP-warehouse-10-20-10-2-1.json'
    # file_dir = f'../logs_for_graphs/2023-01-07--21-03_ALGS-3_RUNS-20_MAP-lt_gallowstemplar_n.json'

    file_dir = f'../logs_for_graphs/2023-01-15--17-06_ALGS-4_RUNS-5_MAP-random-64-64-10.json'
    # file_dir = f'../logs_for_graphs/2023-01-15--18-05_ALGS-4_RUNS-5_MAP-empty-48-48.json'

    # file_dir = '../logs_for_graphs/results_2023-01-04_11-00.json'
    plotter = Plotter()
    show_results(file_dir, plotter=plotter)


if __name__ == '__main__':
    main()
