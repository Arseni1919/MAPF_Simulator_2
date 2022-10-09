import json
from datetime import datetime

import matplotlib.pyplot as plt

from funcs_plotter.plotter import Plotter
from algs.alg_DS_MAPF import run_ds_mapf
from algs.alg_PBS import run_pbs


def show_results(file_dir, plotter, runs_per_n_agents, algs_to_test_dict, n_agents_list, img_png):

    with open(f'{file_dir}', 'r') as openfile:
        # Reading from json file
        json_object = json.load(openfile)
    plotter.plot_big_test(json_object, runs_per_n_agents, algs_to_test_dict, n_agents_list, img_png, is_json=True)
    plt.show()


if __name__ == '__main__':
    file_dir = f'../logs_for_graphs/results_09-10-2022-18-14.json'
    plotter = Plotter()
    algs_to_test_dict = {
        'PBS': run_pbs,
        'DS-MAPF': run_ds_mapf,
    }
    show_results(file_dir,
                 plotter=plotter,
                 runs_per_n_agents=3,
                 algs_to_test_dict=algs_to_test_dict,
                 n_agents_list=[2, 3],
                 img_png=None)
