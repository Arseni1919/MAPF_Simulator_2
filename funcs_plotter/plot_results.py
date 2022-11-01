import json
from datetime import datetime

import matplotlib.pyplot as plt

from funcs_plotter.plotter import Plotter
from algs.alg_DS_MAPF import run_ds_mapf
from algs.alg_PBS import run_pbs


def show_results(file_dir, plotter, algs_to_test_dict):
    """
    to_save_dict = {
        'statistics_dict': statistics_dict,
        'runs_per_n_agents': runs_per_n_agents,
        'n_agents_list': n_agents_list,
        'algs_to_test_names': list(algs_to_test_dict.keys()),
        'img_dir': img_dir

    }
    """
    with open(f'{file_dir}', 'r') as openfile:
        # Reading from json file
        json_object = json.load(openfile)
    statistics_dict = json_object['statistics_dict']
    runs_per_n_agents = json_object['runs_per_n_agents']
    n_agents_list = json_object['n_agents_list']
    img_png = json_object['img_dir']
    algs_to_test_names = json_object['algs_to_test_names']
    plotter.plot_big_test(statistics_dict, runs_per_n_agents, algs_to_test_names, n_agents_list, img_png, is_json=True)
    plt.show()


def main():
    file_dir = f'../logs_for_graphs/results_30-10-2022-22-00.json'
    plotter = Plotter()
    algs_to_test_dict = {
        'PBS': run_pbs,
        'DS-MAPF': run_ds_mapf,
    }
    show_results(file_dir,
                 plotter=plotter,
                 algs_to_test_dict=algs_to_test_dict)


if __name__ == '__main__':
    main()
