from funcs_graph.heuristic_funcs import dist_heuristic, h_func_creator, build_heuristic_for_multiple_targets
from funcs_graph.nodes_from_pic import build_graph_from_png
from funcs_graph.map_dimensions import map_dimensions_dict
# from algs.test_mapf_alg import test_mapf_alg_from_pic
from funcs_plotter.plotter import Plotter
from algs.alg_DS_MAPF import run_ds_mapf
from algs.alg_PBS import run_pbs
from globals import *


def create_statistics_dict(algs_to_test_dict, n_agents_list, runs_per_n_agents):
    return {
        alg_name: {
            n_agents: {
                'success_rate': {run: None for run in range(runs_per_n_agents)},
                'sol_quality': {run: None for run in range(runs_per_n_agents)},
                'runtime': {run: None for run in range(runs_per_n_agents)},
                'iterations_time': {run: None for run in range(runs_per_n_agents)},
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
    img_png = 'warehouse-10-20-10-2-1.png'
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
        a_star_iter_limit
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

            h_dict = build_heuristic_for_multiple_targets(goal_nodes, nodes, map_dim, plotter=inner_plotter, middle_plot=False)
            h_func = h_func_creator(h_dict)
            # h_func = dist_heuristic

            # for at max 5 minutes
            for alg_name, alg in algs_to_test_dict.items():
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
                    # initial_ordering=[]  # for PBS
                )

                # plot + print
                print(f'\n#########################################################')
                print(f'#########################################################')
                print(f'#########################################################')
                print(f'\r[{n_agents} agents][{i_run} run][{alg_name}] -> success_rate: {info["success_rate"]}, result: {result}\n')
                update_statistics_dict(statistics_dict, alg_name, n_agents, i_run, result, info)
                plotter.plot_big_test(statistics_dict, runs_per_n_agents, algs_to_test_dict, n_agents_list, img_png)

    print(f'\nTest finished at: {datetime.now().strftime("%d/%m/%Y %H:%M:%S")}')
    plt.show()


def main():
    algs_to_test_dict = {
        'PBS': run_pbs,
        'DS-MAPF': run_ds_mapf,
    }
    # n_agents_list = [2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
    # n_agents_list = [2, 3, 4, 5, 6, 7, 8, 9, 10]
    # n_agents_list = [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
    n_agents_list = [25, 30, 35, 40]
    runs_per_n_agents = 10
    max_time_per_alg = 1
    random_seed = True
    # random_seed = False
    seed = 197
    plotter = Plotter()
    a_star_iter_limit = 3e3
    big_test(
        algs_to_test_dict=algs_to_test_dict,
        n_agents_list=n_agents_list,
        runs_per_n_agents=runs_per_n_agents,
        max_time_per_alg=max_time_per_alg,
        random_seed=random_seed,
        seed=seed,
        plotter=plotter,
        a_star_iter_limit=a_star_iter_limit
    )


if __name__ == '__main__':
    main()



