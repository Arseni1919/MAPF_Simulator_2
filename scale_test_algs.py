import logging
import cProfile
import pstats
import matplotlib.pyplot as plt

from funcs_graph.heuristic_funcs import dist_heuristic, h_func_creator, build_heuristic_for_multiple_targets
from funcs_graph.heuristic_funcs import parallel_build_heuristic_for_multiple_targets
from funcs_graph.nodes_from_pic import build_graph_nodes, get_dims_from_pic
from funcs_graph.map_dimensions import map_dimensions_dict
# from algs.test_mapf_alg import test_mapf_alg_from_pic
from funcs_plotter.plotter import Plotter
from algs.alg_DS_MAPF import run_ds_mapf
from algs.alg_PBS import run_pbs
from algs.alg_MGM import run_mgm
from globals import *


def save_and_show_results(statistics_dict, file_dir, plotter=None, runs_per_n_agents=None, algs_to_test_dict=None,
                          n_agents_list=None, img_png=None, time_per_alg_limit=None, a_star_iter_limit=None,
                          a_star_calls_limit=None):
    # Serializing json
    to_save_dict = {
        'statistics_dict': statistics_dict,
        'runs_per_n_agents': runs_per_n_agents,
        'n_agents_list': n_agents_list,
        'algs_to_test_names': list(algs_to_test_dict.keys()),
        'img_dir': img_png,
        'time_per_alg_limit': time_per_alg_limit,
        'a_star_iter_limit': a_star_iter_limit,
        'a_star_calls_limit': a_star_calls_limit,

    }
    json_object = json.dumps(to_save_dict, indent=4)
    # Writing to sample.json
    # file_dir = f'logs_for_graphs/results_{datetime.now().strftime("%d-%m-%Y-%H-%M")}.json'
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
                'a_star_runtimes': [],
                'a_star_n_closed': [],
                'n_closed_per_run': [],
                'n_agents_conf': [],
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

        if 'a_star_runtimes' in info:
            statistics_dict[alg_name][n_agents]['a_star_runtimes'].extend(info['a_star_runtimes'])

        if 'a_star_n_closed' in info:
            statistics_dict[alg_name][n_agents]['a_star_n_closed'].extend(info['a_star_n_closed'])
            statistics_dict[alg_name][n_agents]['n_closed_per_run'].append(sum(info['a_star_n_closed']))
        if 'n_agents_conf' in info:
            statistics_dict[alg_name][n_agents]['n_agents_conf'].extend(info['n_agents_conf'])


def set_seed(random_seed, seed):
    if random_seed:
        seed = random.choice(range(1000))
    random.seed(seed)
    np.random.seed(seed)
    print(f'SEED: {seed}')


def get_map_nodes():
    # img_dir = 'warehouse-10-20-10-2-1.map'  # 63-161
    # img_dir = 'random-64-64-10.map'  # 64-64
    img_dir = 'lt_gallowstemplar_n.map'  # 180-251


    # img_dir = 'random-32-32-10.map'  # 32-32
    # img_dir = 'room-64-64-8.map'  # 64-64
    # img_dir = 'random-64-64-20.map'  # 64-64
    # img_dir = 'warehouse-10-20-10-2-2.map'  # 84-170
    # img_dir = 'warehouse-20-40-10-2-1.map'  # 123-321
    # img_dir = 'maze-128-128-2.map'  # 128-128
    # img_dir = 'ht_chantry.map'  # 141-162
    # img_dir = 'ost003d.map'  # 194-194
    # img_dir = 'lak303d.map'  # 194-194
    # img_dir = 'warehouse-20-40-10-2-2.map'  # 164-340
    # img_dir = 'Berlin_1_256.map'  # 256-256
    # img_dir = 'den520d.map'  # 257-256
    # img_dir = 'ht_mansion_n.map'  # 270-133
    # img_dir = 'brc202d.map'  # 481-530

    # img_dir = 'lak108d.png'
    # img_dir = 'lak109d.png'
    # img_dir = '19_20_warehouse.png'
    # img_dir = '22_22_blank_grid.png'
    # img_dir = '22_22_blank_grid_rate_0.1.png'
    # img_dir = 'warehouse-10-20-10-2-1.png'
    # img_dir = 'den101d.png'
    # img_dir = 'rmtst.png'
    # img_dir = 'lak505d.png'
    # img_dir = 'lak503d.png'
    # img_dir = 'ost003d.png'
    # img_dir = 'brc202d.png'
    # img_dir = 'den520d.png'

    map_dim = get_dims_from_pic(img_dir=img_dir, path='maps')
    nodes, nodes_dict = build_graph_nodes(img_dir=img_dir, path='maps', show_map=False)
    return nodes, nodes_dict, map_dim, img_dir


def big_test(
        algs_to_test_dict: dict,
        n_agents_list: list,
        runs_per_n_agents: int,
        time_per_alg_limit,
        random_seed: bool,
        seed: int,
        plotter,
        a_star_iter_limit,
        a_star_calls_limit,
        to_save_results,
        file_dir,
        profiler=None,
):
    print(f'\nTest started at: {datetime.now().strftime("%d/%m/%Y %H:%M:%S")}')

    # seed
    set_seed(random_seed, seed)

    # get nodes and dimensions from image
    nodes, nodes_dict, map_dim, img_png = get_map_nodes()
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
                    max_time=time_per_alg_limit,
                    a_star_iter_limit=a_star_iter_limit,
                    a_star_calls_limit=a_star_calls_limit,
                    # initial_ordering=[]  # for PBS
                    **params
                )

                # plot + print
                print(f'\n#########################################################')
                print(f'#########################################################')
                print(f'#########################################################')
                print(f'#########################################################')
                print(f'#########################################################')
                print(f'#########################################################')
                print(f'#########################################################')
                print(f'#########################################################')
                print(f'#########################################################')
                print(f'\r[{n_agents} agents][{i_run} run][{alg_name}] -> success_rate: {info["success_rate"]}\n')
                update_statistics_dict(statistics_dict, alg_name, n_agents, i_run, result, info)
                if i_run % 2 == 0:
                    plotter.plot_big_test(statistics_dict, runs_per_n_agents, list(algs_to_test_dict.keys()), n_agents_list, img_png)

        if to_save_results:
            save_and_show_results(statistics_dict, file_dir, None, runs_per_n_agents, algs_to_test_dict, n_agents_list, img_png, time_per_alg_limit, a_star_iter_limit, a_star_calls_limit)
            print('Results saved.')

    print(f'\nTest finished at: {datetime.now().strftime("%d/%m/%Y %H:%M:%S")}')


def main():
    logging.basicConfig(format='%(asctime)s - %(message)s', datefmt='%d/%m/%Y %H:%M:%S', level=logging.INFO)

    algs_to_test_dict = {
        # 'PBS': (run_pbs, {}),
        'MGM': (run_mgm, {}),
        # 'DS-0.2': (run_ds_mapf, {'alpha': 0.2, 'decision_type': 'simple'}),
        # 'DS-0.5': (run_ds_mapf, {'alpha': 0.5, 'decision_type': 'simple'}),
        # 'DS-0.8': (run_ds_mapf, {'alpha': 0.8, 'decision_type': 'simple'}),
        'DS-min_prev_1': (run_ds_mapf, {'alpha': 0.5, 'decision_type': 'min_prev_1', 'limit_type': 'simple'}),
        # 'DS-max_prev_1': (run_ds_mapf, {'alpha': 0.5, 'decision_type': 'max_prev_1', 'limit_type': 'simple'}),
        # 'DS-index_1': (run_ds_mapf, {'alpha': 0.5, 'decision_type': 'index_1', 'limit_type': 'simple'}),
        'DS-min_prev_2': (run_ds_mapf, {'decision_type': 'min_prev_2', 'limit_type': 'simple'}),
        'DS-index_2': (run_ds_mapf, {'decision_type': 'index_2', 'limit_type': 'simple'}),
    }

    # n_agents_list = [2, 3, 4, 5]
    # n_agents_list = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
    # n_agents_list = [2, 3, 4, 5, 6, 7, 8, 9, 10]
    # n_agents_list = [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
    # n_agents_list = [10, 20, 30, 40]
    # n_agents_list = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]  # !!!!!!!!!!!!!!!!!
    # n_agents_list = [20, 30, 40, 50, 60, 70, 80, 90, 100]
    # n_agents_list = [50, 60, 70, 80, 90, 100]
    n_agents_list = [100, 120, 140, 160, 180, 200]
    # n_agents_list = [100, 150, 200, 250]

    # runs_per_n_agents = 50  # !!!!!!!!!!!!!!!!!
    # runs_per_n_agents = 20
    # runs_per_n_agents = 10
    runs_per_n_agents = 5

    # time_per_alg_limit = 1  # According to PBS paper!
    # time_per_alg_limit = 3
    # time_per_alg_limit = 10
    time_per_alg_limit = 50

    random_seed = True
    # random_seed = False

    seed = 197
    plotter = Plotter()
    a_star_iter_limit = 1e9

    # a_star_calls_limit = 100
    a_star_calls_limit = 500
    # a_star_calls_limit = 1e100

    to_save_results = True
    # to_save_results = False
    file_dir = f'logs_for_graphs/results_{datetime.now().strftime("%Y-%m-%d_%H-%M")}.json'

    # profiler = None
    profiler = cProfile.Profile()

    if profiler:
        profiler.enable()
    big_test(
        algs_to_test_dict=algs_to_test_dict,
        n_agents_list=n_agents_list,
        runs_per_n_agents=runs_per_n_agents,
        time_per_alg_limit=time_per_alg_limit,
        random_seed=random_seed,
        seed=seed,
        plotter=plotter,
        a_star_iter_limit=a_star_iter_limit,
        a_star_calls_limit=a_star_calls_limit,
        to_save_results=to_save_results,
        file_dir=file_dir,
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



