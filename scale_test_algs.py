import logging
import cProfile
import pstats
import matplotlib.pyplot as plt

from funcs_graph.heuristic_funcs import dist_heuristic, h_func_creator, build_heuristic_for_multiple_targets
from funcs_graph.heuristic_funcs import parallel_build_heuristic_for_multiple_targets
from funcs_graph.nodes_from_pic import build_graph_from_png
from funcs_graph.map_dimensions import map_dimensions_dict
# from algs.test_mapf_alg import test_mapf_alg_from_pic
from funcs_plotter.plotter import Plotter
from algs.alg_DS_MAPF import run_ds_mapf
from algs.alg_PBS import run_pbs
from globals import *


def save_and_show_results(statistics_dict, plotter=None, runs_per_n_agents=None, algs_to_test_dict=None, n_agents_list=None, img_png=None):
    # Serializing json
    to_save_dict = {
        'statistics_dict': statistics_dict,
        'runs_per_n_agents': runs_per_n_agents,
        'n_agents_list': n_agents_list,
        'algs_to_test_names': list(algs_to_test_dict.keys()),
        'img_png': img_png

    }
    json_object = json.dumps(to_save_dict, indent=4)
    # Writing to sample.json
    file_dir = f'logs_for_graphs/results_{datetime.now().strftime("%d-%m-%Y-%H-%M")}.json'
    with open(file_dir, "w") as outfile:
        outfile.write(json_object)
    # Results saved.
    if plotter:
        with open(f'{file_dir}', 'r') as openfile:
            # Reading from json file
            json_object = json.load(openfile)
        plotter.plot_big_test(json_object['statistics_dict'], runs_per_n_agents, list(algs_to_test_dict.keys()), n_agents_list, img_png, is_json=True)


def create_statistics_dict(algs_to_test_dict, n_agents_list, runs_per_n_agents):
    return {
        alg_name: {
            n_agents: {
                'success_rate': {run: None for run in range(runs_per_n_agents)},
                'sol_quality': {run: None for run in range(runs_per_n_agents)},
                'runtime': {run: None for run in range(runs_per_n_agents)},
                'iterations_time': {run: None for run in range(runs_per_n_agents)},
                'a_star_calls_counter': {run: None for run in range(runs_per_n_agents)},
                'a_star_calls_dist_counter': {run: None for run in range(runs_per_n_agents)},
            } for n_agents in n_agents_list
        } for alg_name, _ in algs_to_test_dict.items()
    }


def update_statistics_dict(statistics_dict, alg_name, n_agents, i_run, result, info):
    statistics_dict[alg_name][n_agents]['success_rate'][i_run] = info['success_rate']
    if info['success_rate'] == 1:
        statistics_dict[alg_name][n_agents]['sol_quality'][i_run] = info['sol_quality']
        statistics_dict[alg_name][n_agents]['runtime'][i_run] = info['runtime']

        if 'iterations_time' in info:
            statistics_dict[alg_name][n_agents]['iterations_time'][i_run] = info['iterations_time']

        if 'a_star_calls_counter' in info:
            statistics_dict[alg_name][n_agents]['a_star_calls_counter'][i_run] = info['a_star_calls_counter']

        if 'a_star_calls_dist_counter' in info:
            statistics_dict[alg_name][n_agents]['a_star_calls_dist_counter'][i_run] = info['a_star_calls_dist_counter']


def set_seed(random_seed, seed):
    if random_seed:
        seed = random.choice(range(1000))
    random.seed(seed)
    np.random.seed(seed)
    print(f'SEED: {seed}')


def get_nodes_from_pic():
    # img_png = 'lak108d.png'
    # img_png = 'lak109d.png'
    # img_png = '19_20_warehouse.png'
    img_png = '22_22_blank_grid.png'
    # img_png = '22_22_blank_grid_rate_0.1.png'
    # img_png = 'warehouse-10-20-10-2-1.png'
    # img_png = 'den101d.png'
    # img_png = 'rmtst.png'
    # img_png = 'lak505d.png'
    # img_png = 'lak503d.png'
    # img_png = 'ost003d.png'
    # img_png = 'brc202d.png'
    # img_png = 'den520d.png'
    map_dim = map_dimensions_dict[img_png]
    nodes, nodes_dict = build_graph_from_png(img_png=img_png, path='maps', show_map=False)
    return nodes, nodes_dict, map_dim, img_png


def big_test(
        algs_to_test_dict: dict,
        n_agents_list: list,
        runs_per_n_agents: int,
        max_time_per_alg,
        random_seed: bool,
        seed: int,
        plotter,
        a_star_iter_limit,
        a_star_calls_limit,
        to_save_results,
        profiler=None,
):
    print(f'\nTest started at: {datetime.now().strftime("%d/%m/%Y %H:%M:%S")}')

    # seed
    set_seed(random_seed, seed)

    # get nodes and dimensions from image
    nodes, nodes_dict, map_dim, img_png = get_nodes_from_pic()
    # inner_plotter = Plotter(map_dim=map_dim)
    inner_plotter = None

    # for plotter
    statistics_dict = create_statistics_dict(algs_to_test_dict=algs_to_test_dict, n_agents_list=n_agents_list,
                                             runs_per_n_agents=runs_per_n_agents)

    # for num of agents
    for n_agents in n_agents_list:

        # for same starts and goals
        for i_run in range(runs_per_n_agents):
            sample_nodes = random.sample(nodes, 2 * n_agents)
            start_nodes = sample_nodes[:n_agents]
            goal_nodes = sample_nodes[n_agents:]
            # h_dict = build_heuristic_for_multiple_targets(goal_nodes, nodes, map_dim, plotter=inner_plotter, middle_plot=False)
            h_dict = parallel_build_heuristic_for_multiple_targets(goal_nodes, nodes, map_dim, plotter=inner_plotter, middle_plot=False)
            h_func = h_func_creator(h_dict)
            # h_func = dist_heuristic

            # for at max 5 minutes
            for alg_name, (alg, params) in algs_to_test_dict.items():
                result, info = alg(
                    start_nodes=start_nodes,
                    goal_nodes=goal_nodes,
                    nodes=nodes,
                    nodes_dict=nodes_dict,
                    h_func=h_func,
                    plotter=inner_plotter,
                    middle_plot=False,
                    final_plot=False,
                    max_time=max_time_per_alg,
                    a_star_iter_limit=a_star_iter_limit,
                    a_star_calls_limit=a_star_calls_limit,
                    # initial_ordering=[]  # for PBS
                    **params
                )

                # plot + print
                print(f'\n#########################################################')
                print(f'#########################################################')
                print(f'#########################################################')
                print(f'\r[{n_agents} agents][{i_run} run][{alg_name}] -> success_rate: {info["success_rate"]}, result: {result}\n')
                update_statistics_dict(statistics_dict, alg_name, n_agents, i_run, result, info)
                plotter.plot_big_test(statistics_dict, runs_per_n_agents, list(algs_to_test_dict.keys()), n_agents_list, img_png)

    print(f'\nTest finished at: {datetime.now().strftime("%d/%m/%Y %H:%M:%S")}')

    if to_save_results:
        save_and_show_results(statistics_dict, plotter, runs_per_n_agents, algs_to_test_dict, n_agents_list, img_png)
        print('Results saved.')


def main():
    logging.basicConfig(format='%(asctime)s - %(message)s', datefmt='%d/%m/%Y %H:%M:%S', level=logging.INFO)
    algs_to_test_dict = {
        'PBS': (run_pbs, {}),
        # 'DS-MAPF-1': (run_ds_mapf, {'alpha': 1.0}),
        # 'DS-MAPF-0.8': (run_ds_mapf, {'alpha': 0.8}),
        'DS-MAPF-0.7': (run_ds_mapf, {'alpha': 0.7}),
        'DS-MAPF-0.6': (run_ds_mapf, {'alpha': 0.6}),
        'DS-MAPF-0.5': (run_ds_mapf, {'alpha': 0.5}),
        # 'DS-MAPF-0.4': (run_ds_mapf, {'alpha': 0.4}),
        # 'DS-MAPF-0.2': (run_ds_mapf, {'alpha': 0.2}),
    }
    # n_agents_list = [2, 3]
    # n_agents_list = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
    # n_agents_list = [2, 3, 4, 5, 6, 7, 8, 9, 10]
    # n_agents_list = [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
    # n_agents_list = [25, 30, 35, 40]
    n_agents_list = [20, 30, 40, 50, 60, 70, 80, 90, 100]
    runs_per_n_agents = 50
    max_time_per_alg = 1
    # max_time_per_alg = 50  # According to PBS paper!
    # random_seed = True
    random_seed = False
    seed = 197
    plotter = Plotter()
    a_star_iter_limit = 3e3
    # a_star_calls_limit = 200
    a_star_calls_limit = 1e100
    to_save_results = True
    # to_save_results = False

    # profiler = None
    profiler = cProfile.Profile()
    if profiler:
        profiler.enable()
    big_test(
        algs_to_test_dict=algs_to_test_dict,
        n_agents_list=n_agents_list,
        runs_per_n_agents=runs_per_n_agents,
        max_time_per_alg=max_time_per_alg,
        random_seed=random_seed,
        seed=seed,
        plotter=plotter,
        a_star_iter_limit=a_star_iter_limit,
        a_star_calls_limit=a_star_calls_limit,
        to_save_results=to_save_results,
        profiler=profiler,
    )
    if profiler:
        profiler.disable()
        stats = pstats.Stats(profiler).sort_stats('cumtime')
        stats.dump_stats('stats/results_scale_experiments.pstat')
        print('Profile saved to stats/results_scale_experiments.pstat.')
    plt.show()


if __name__ == '__main__':
    main()



