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
from algs.alg_MGDS import run_mgds
from algs.alg_MGM_classic import run_mgm_classic
from algs.alg_PP import run_pp
from algs.alg_a_star import a_star
from algs.alg_a_star_short import a_star_short
# from algs.alg_depth_first_a_star import df_a_star
from globals import *


def save_and_show_results(to_save_dict, file_dir, plotter=None, runs_per_n_agents=None, algs_to_test_dict=None,
                          n_agents_list=None, img_png=None):
    # Serializing json
    json_object = json.dumps(to_save_dict, indent=4)
    with open(file_dir, "w") as outfile:
        outfile.write(json_object)
    # Results saved.
    if plotter:
        with open(f'{file_dir}', 'r') as openfile:
            # Reading from json file
            json_object = json.load(openfile)
        plotter.plot_big_test(json_object['stats_dict'], runs_per_n_agents, algs_to_test_dict, n_agents_list, img_png,
                              is_json=True, n_agents=max(n_agents_list))


def create_to_save_dict(algs_to_test_dict, n_agents_list, runs_per_n_agents, **kwargs):
    stats_dict = {
        alg_name: {
            n_agents: {
                'success_rate': {run: None for run in range(runs_per_n_agents)},
                'sol_quality': {run: None for run in range(runs_per_n_agents)},
                'runtime': {run: None for run in range(runs_per_n_agents)},
                'dist_runtime': {run: None for run in range(runs_per_n_agents)},
                'a_star_calls_counter': {run: None for run in range(runs_per_n_agents)},
                'a_star_calls_counter_dist': [],
                'a_star_runtimes': [],
                'a_star_n_closed': [],
                'a_star_n_closed_dist': [],
                'n_closed_per_run': [],
                'n_agents_conf': [],
                'a_star_calls_per_agent': [],
                'n_messages_per_agent': [],
                'confs_per_iter': [],
            } for n_agents in n_agents_list
        } for alg_name, _ in algs_to_test_dict.items()
    }
    to_save_dict = {
        'stats_dict': stats_dict,
        'runs_per_n_agents': runs_per_n_agents,
        'n_agents_list': n_agents_list,
        'algs_to_test_dict': {k: (None, {
            v2k: v2v for v2k, v2v in v2.items() if v2k not in ['a_star_func']
        }) for k, (v1, v2) in algs_to_test_dict.items()},
    }
    to_save_dict.update(kwargs)
    return to_save_dict


def update_statistics_dict(stats_dict, alg_name, n_agents, i_run, result, alg_info):
    stats_dict[alg_name][n_agents]['success_rate'][i_run] = alg_info['success_rate']
    if alg_info['success_rate'] == 1:
        stats_dict[alg_name][n_agents]['sol_quality'][i_run] = alg_info['sol_quality']
        stats_dict[alg_name][n_agents]['runtime'][i_run] = alg_info['runtime']

        if 'dist_runtime' in alg_info:
            stats_dict[alg_name][n_agents]['dist_runtime'][i_run] = alg_info['dist_runtime']
            stats_dict[alg_name][n_agents]['a_star_n_closed_dist'].append(alg_info['a_star_n_closed_dist'])
            stats_dict[alg_name][n_agents]['a_star_calls_counter_dist'].append(alg_info['a_star_calls_counter_dist'])

        if 'a_star_calls_counter' in alg_info:
            stats_dict[alg_name][n_agents]['a_star_calls_counter'][i_run] = alg_info['a_star_calls_counter']

        if 'a_star_runtimes' in alg_info:
            stats_dict[alg_name][n_agents]['a_star_runtimes'].extend(alg_info['a_star_runtimes'])

        if 'a_star_n_closed' in alg_info:
            stats_dict[alg_name][n_agents]['a_star_n_closed'].extend(alg_info['a_star_n_closed'])
            stats_dict[alg_name][n_agents]['n_closed_per_run'].append(sum(alg_info['a_star_n_closed']))

        if 'n_agents_conf' in alg_info:
            stats_dict[alg_name][n_agents]['n_agents_conf'].extend(alg_info['n_agents_conf'])

        if 'a_star_calls_per_agent' in alg_info:
            stats_dict[alg_name][n_agents]['a_star_calls_per_agent'].extend(alg_info['a_star_calls_per_agent'])

        if 'n_messages_per_agent' in alg_info:
            stats_dict[alg_name][n_agents]['n_messages_per_agent'].extend(alg_info['n_messages_per_agent'])

        if 'confs_per_iter' in alg_info:
            stats_dict[alg_name][n_agents]['confs_per_iter'] = alg_info['confs_per_iter']


def set_seed(random_seed, seed):
    if random_seed:
        seed = random.choice(range(1000))
    random.seed(seed)
    np.random.seed(seed)
    print(f'SEED: {seed}')


def get_map_nodes(only_name=False):
    # img_dir = 'empty-48-48.map'  # 48-48
    img_dir = 'random-64-64-10.map'  # 64-64
    # img_dir = 'warehouse-10-20-10-2-1.map'  # 63-161
    # img_dir = 'lt_gallowstemplar_n.map'  # 180-251

    # img_dir = 'random-32-32-10.map'  # 32-32
    # img_dir = 'orz900d.map'  # 656-1491

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
    if only_name:
        return img_dir[:-4]
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
        a_star_closed_nodes_limit,
        to_save_results,
        file_dir,
        profiler=None,
):
    print(f'\nTest started at: {datetime.now().strftime("%d/%m/%Y %H:%M:%S")}')

    # seed
    set_seed(random_seed, seed)

    # get nodes and dimensions from image
    nodes, nodes_dict, map_dim, img_dir = get_map_nodes()
    # inner_plotter = Plotter(map_dim=map_dim)
    inner_plotter = None

    # for plotter
    to_save_dict = create_to_save_dict(
        algs_to_test_dict=algs_to_test_dict,
        n_agents_list=n_agents_list,
        runs_per_n_agents=runs_per_n_agents,
        img_dir=img_dir,
        time_per_alg_limit=time_per_alg_limit,
        a_star_iter_limit=a_star_iter_limit,
        a_star_calls_limit=a_star_calls_limit,
    )
    stats_dict = to_save_dict['stats_dict']

    # for num of agents
    for n_agents in n_agents_list:

        # for same starts and goals
        for i_run in range(runs_per_n_agents):
            sample_nodes = random.sample(nodes, 2 * n_agents)
            start_nodes = sample_nodes[:n_agents]
            goal_nodes = sample_nodes[n_agents:]
            # h_dict = build_heuristic_for_multiple_targets(goal_nodes, nodes, map_dim, plotter=inner_plotter, middle_plot=False)
            h_dict = parallel_build_heuristic_for_multiple_targets(goal_nodes, nodes, map_dim, plotter=inner_plotter,
                                                                   middle_plot=False)
            h_func = h_func_creator(h_dict)
            # h_func = dist_heuristic

            # for at max 5 minutes
            for alg_name, (alg, params) in algs_to_test_dict.items():
                result, alg_info = alg(
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
                    a_star_closed_nodes_limit=a_star_closed_nodes_limit,
                    # initial_ordering=[]  # for PBS
                    alg_name=alg_name,
                    **params,
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
                print(f'\r[{n_agents} agents][{i_run} run][{alg_name}] -> success_rate: {alg_info["success_rate"]}\n')
                update_statistics_dict(stats_dict, alg_name, n_agents, i_run, result, alg_info)
                if i_run % 1 == 0:
                    plotter.plot_big_test(stats_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, img_dir,
                                          n_agents=n_agents)

        if to_save_results:
            save_and_show_results(to_save_dict, file_dir, None, runs_per_n_agents, algs_to_test_dict, n_agents_list,
                                  img_dir)
            print('Results are saved.')

    print(f'\nTest finished at: {datetime.now().strftime("%d/%m/%Y %H:%M:%S")}')


def main():
    logging.basicConfig(format='%(asctime)s - %(message)s', datefmt='%d/%m/%Y %H:%M:%S', level=logging.INFO)
    algs_to_test_dict = {
        # 'PBS': (run_pbs, {
        #     'a_star_func': a_star,
        #     'limit_type': 'norm_time',
        #     'dist': False,
        #     'color': 'tab:purple',
        # }),
        'PP': (run_pp, {
            'a_star_func': a_star,
            'limit_type': 'norm_a_star_closed',
            'dist': False,
            'color': 'tab:blue',
        }),
        # 'DSA': (run_ds_mapf, {
        #     'a_star_func': a_star,
        #     'limit_type': 'dist_a_star_closed',
        #     'decision_type': 'simple',
        #     'alpha': 0.5,
        #     'dist': True,
        #     'color': 'tab:brown',
        # }),
        'SDS': (run_ds_mapf, {
            'a_star_func': a_star,
            'limit_type': 'dist_a_star_closed',
            'decision_type': 'min_prev_2',
            'dist': True,
            'color': 'tab:orange',
        }),
        # 'MGM': (run_mgm_classic, {
        #     'a_star_func': a_star,
        #     'limit_type': 'dist_time',
        #     'dist': True,
        #     'color': 'tab:olive',
        # }),
        'MGDS': (run_mgds, {'a_star_func': a_star,
                            'limit_type': 'dist_a_star_closed',
                            'gain_type': 'rank',
                            'alpha': 0.9,
                            'dist': True,
                            'color': 'tab:green'}),

        # 'MGDS-0.95': (run_mgds, {'a_star_func': a_star,
        #                          'limit_type': 'dist_a_star_closed',
        #                          'gain_type': 'rank',
        #                          'alpha': 0.95,
        #                          'dist': True,
        #                          'color': 'tab:green'}),
        # 'MGDS-0.7': (run_mgds, {'a_star_func': a_star,
        #                         'limit_type': 'dist_a_star_closed',
        #                         'gain_type': 'rank',
        #                         'alpha': 0.7,
        #                         'dist': True}),
        # 'MGDS-0.5': (run_mgds, {'a_star_func': a_star,
        #                         'limit_type': 'dist_a_star_closed',
        #                         'gain_type': 'rank',
        #                         'alpha': 0.5,
        #                         'dist': True}),
        # 'MGDS-0.3': (run_mgds, {'a_star_func': a_star,
        #                         'limit_type': 'dist_a_star_closed',
        #                         'gain_type': 'rank',
        #                         'alpha': 0.3,
        #                         'dist': True}),
        # 'MGDS-0.1': (run_mgds, {'a_star_func': a_star,
        #                         'limit_type': 'dist_a_star_closed',
        #                         'gain_type': 'rank',
        #                         'alpha': 0.1,
        #                         'dist': True}),
        # 'MGDS_confs_d': (run_mgm, {'a_star_func': a_star, 'limit_type': 'dist_time', 'gain_type': 'sum_of_confs'}),
        # 'PBS_a2': (run_pbs, {'a_star_func': df_a_star}),
        # 'PP': (run_pp, {'a_star_mode': 'simple', 'a_star_func': a_star, 'limit_type': 'norm_time'}),
        # 'PP': (run_pp, {'a_star_func': a_star, 'limit_type': 'norm_a_star_closed'}),
        # 'PP-short': (run_pp, {'a_star_func': a_star_short, 'limit_type': 'norm_time'}),
        # 'PP_a2': (run_pp, {'a_star_mode': 'simple', 'a_star_func': df_a_star}),
        # 'PP_f': (run_pp, {'a_star_mode': 'fast'}),
        # 'MGM_d': (run_mgm, {'a_star_func': a_star, 'limit_type': 'norm_time'}),
        # 'MGM_d': (run_mgm, {'a_star_func': a_star, 'limit_type': 'dist_time'}),
        # 'MGM_d': (run_mgm, {'a_star_func': a_star, 'limit_type': 'norm_a_star_closed'}),
        # 'MGM_d': (run_mgm, {'a_star_func': a_star, 'limit_type': 'dist_a_star_closed'}),
        # 'MGM_d_a2': (run_mgm, {'a_star_func': df_a_star}),
        # 'DS-0.2': (run_ds_mapf, {'alpha': 0.2, 'decision_type': 'simple'}),
        # 'DS-0.5': (run_ds_mapf, {'alpha': 0.5, 'decision_type': 'simple'}),
        # 'DS-0.8': (run_ds_mapf, {'alpha': 0.8, 'decision_type': 'simple'}),
        # 'DS-min_prev_1': (run_ds_mapf, {'alpha': 0.5, 'decision_type': 'min_prev_1', 'limit_type': 'simple'}),
        # 'DS-max_prev_1': (run_ds_mapf, {'alpha': 0.5, 'decision_type': 'max_prev_1', 'limit_type': 'simple'}),
        # 'DS-index_1': (run_ds_mapf, {'alpha': 0.5, 'decision_type': 'index_1', 'limit_type': 'simple'}),
        # 'DS-min_2_d': (run_ds_mapf, {'decision_type': 'min_prev_2', 'limit_type': 'norm_time', 'a_star_func': a_star}),
        # 'DS-min_2_d': (run_ds_mapf, {'decision_type': 'min_prev_2', 'limit_type': 'dist_time', 'a_star_func': a_star}),
        # 'DS-min_2_d': (run_ds_mapf, {'decision_type': 'min_prev_2', 'limit_type': 'norm_a_star_closed', 'a_star_func': a_star}),
        # 'DS-min_2_d': (run_ds_mapf, {'a_star_func': a_star, 'decision_type': 'min_prev_2', 'limit_type': 'dist_a_star_closed'}),
        # 'DS-short_d': (run_ds_mapf, {'a_star_func': a_star_short, 'decision_type': 'min_prev_2', 'limit_type': 'dist_time'}),
        # 'DS-index_2_d': (run_ds_mapf, {'decision_type': 'index_2', 'limit_type': 'simple'}),
    }

    # n_agents_list = [2, 3, 4, 5]
    # n_agents_list = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
    # n_agents_list = [2, 3, 4, 5, 6, 7, 8, 9, 10]
    # n_agents_list = [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
    # n_agents_list = [10, 20, 30, 40]
    # n_agents_list = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]  # !!!!!!!!!!!!!!!!!
    n_agents_list = [20, 40, 60, 80, 100, 120, 140, 160, 180, 200]  # !!!!!!!!!!!!!!!!!
    # n_agents_list = [20, 60, 100, 140, 180, 220, 260, 300, 340]
    # n_agents_list = [20, 30, 40, 50, 60, 70, 80, 90, 100]
    # n_agents_list = [50, 60, 70, 80, 90, 100]
    # n_agents_list = [100, 120, 140, 160, 180, 200]
    # n_agents_list = [100, 120, 140, 160, 180, 200, 220, 240, 260, 280, 300]
    # n_agents_list = [100, 150, 200, 250, 300, 350, 400, 450, 500]
    # n_agents_list = [150, 200, 250, 300, 350, 400]
    # n_agents_list = [300, 350, 400, 450, 500]
    # n_agents_list = [250, 300, 350, 400, 450, 500, 550]

    # runs_per_n_agents = 50
    # runs_per_n_agents = 25
    runs_per_n_agents = 20  # !!!!!!!!!!!!!!!!!
    # runs_per_n_agents = 10
    # runs_per_n_agents = 5
    # runs_per_n_agents = 2
    # runs_per_n_agents = 3

    random_seed = True
    # random_seed = False
    seed = 116

    # ------------------------------ LIMITS ------------------------------ #
    # time_per_alg_limit = 1  # According to PBS paper!
    # time_per_alg_limit = 0.1
    # time_per_alg_limit = 3
    time_per_alg_limit = 10
    # time_per_alg_limit = 50

    # a_star_calls_limit = 100
    # a_star_calls_limit = 500
    # a_star_calls_limit = 1500
    a_star_calls_limit = 1e100

    # a_star_closed_nodes_limit = 1e100
    # a_star_closed_nodes_limit = 1e7
    # a_star_closed_nodes_limit = 1e6
    a_star_closed_nodes_limit = 5e5

    a_star_iter_limit = 1e100
    # ---------------------------- END LIMITS --------------------------- #

    plotter = Plotter()

    to_save_results = True
    # to_save_results = False
    file_dir = f'logs_for_graphs/{datetime.now().strftime("%Y-%m-%d--%H-%M")}_ALGS-{len(algs_to_test_dict)}_RUNS-{runs_per_n_agents}_MAP-{get_map_nodes(True)}.json'

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
        a_star_closed_nodes_limit=a_star_closed_nodes_limit,
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
