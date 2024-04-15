import math
import random

from functions import *
import cProfile
import pstats
from typing import *
from algs.test_mapf_alg import test_mapf_alg_from_pic
from algs.metrics import c_v_check_for_agent, c_e_check_for_agent, build_constraints, \
    limit_is_crossed, get_agents_in_conf, check_plan, get_alg_info_dict, iteration_print
from algs.metrics import just_check_k_step_plans, just_check_plans
from simulator_objects import Node


class BnBAgent:
    def __init__(self, index, start_node, goal_node, nodes, nodes_dict, h_func):
        self.index = index
        self.start_node: Node = start_node
        self.curr_node: Node = start_node
        self.goal_node: Node = goal_node
        self.nodes: List[Node] = nodes
        self.nodes_dict: Dict[str, Node] = nodes_dict
        self.h_func = h_func
        self.path = []

        # stats
        self.stats_n_closed = 0
        self.stats_n_calls = 0
        self.stats_runtime = 0
        self.stats_n_messages = 0
        self.stats_n_step_m = 0
        self.stats_n_step_m_list = []
        self.stats_nei_list = []
        # nei
        self.nei_list: List[Self] = []
        self.nei_dict: Dict[str, Self] = {}
        self.nei_paths_dict = {}
        # bnb
        self.parent: Self = None
        self.pseudo_parents: List[Self] = []
        self.children: List[Self] = []
        self.pseudo_children: List[Self] = []
        self.visited: bool = False
        self.level: int = -1
        self.bnb_next_n_deque = deque(self.curr_node.neighbours)
        self.bnb_next_a_deque = deque()
        self.next_n_cost_cpa_dict = {}
        self.curr_next_n = None
        self.unary_c = {}

    def clear_nei(self):
        self.nei_list = []
        self.nei_dict = {}
        self.nei_paths_dict = {}
        # bnb
        self.parent = None
        self.pseudo_parents: List[Self] = []
        self.children = []
        self.pseudo_children: List[Self] = []
        self.visited = False
        self.level = -1
        self.bnb_next_n_deque = deque(self.curr_node.neighbours)
        self.bnb_next_a_deque = deque()
        self.next_n_cost_cpa_dict = {}
        self.curr_next_n = None
        # update unary table
        self.unary_c = {}
        random.shuffle(self.curr_node.neighbours)
        # curr_v = self.h_func(self.curr_node, self.goal_node)
        for nei_node_name in self.curr_node.neighbours:
            nei_node = self.nodes_dict[nei_node_name]
            next_v = self.h_func(nei_node, self.goal_node)
            self.unary_c[nei_node_name] = next_v

    def add_nei(self, agent: Self):
        self.nei_list.append(agent)
        self.nei_dict[agent.name] = agent

    def __eq__(self, other: Self) -> bool:
        return self.index == other.index

    @property
    def name(self):
        return f'agent_{self.index}'

    @property
    def curr_node_name(self):
        return self.curr_node.xy_name

    @property
    def goal_node_name(self):
        return self.goal_node.xy_name

    @property
    def path_names(self):
        return [n.xy_name for n in self.path]


def plot_edges(agents: List[BnBAgent]) -> None:
    g = graphviz.Graph('agents', format='png')
    for agent in agents:
        # single
        if len(agent.children) == 0:
            g.node(agent.name)
            continue
        # child
        for child in agent.children:
            g.edge(agent.name, child.name)
        for nei in agent.nei_list:
            # pseudo-child
            if nei not in agent.children and agent.level < nei.level:
                g.edge(agent.name, nei.name, label='PC')
    g.view()


def run_dfs(agent: BnBAgent, parent=None) -> None:
    agent.visited = True
    agent.parent = parent
    if parent:
        agent.level = parent.level + 1
    for nei in agent.nei_list:
        if not nei.visited:
            run_dfs(nei, parent=agent)
            agent.children.append(nei)


def get_pseudo_trees(agents: List[BnBAgent]) -> list:
    pseudo_tree_list = []
    # shuffle
    randomly_ordered_agents: List[BnBAgent] = agents[:]
    random.shuffle(randomly_ordered_agents)

    # clear
    for agent in randomly_ordered_agents:
        agent.clear_nei()

    # create edges
    for agent1, agent2 in combinations(randomly_ordered_agents, 2):
        if agent1.curr_node.xy_name in agent2.curr_node.neighbours:
            agent1.add_nei(agent2)
            agent2.add_nei(agent1)

    # DFSs
    for agent in randomly_ordered_agents:
        if agent.visited:
            continue
        agent.level = 1
        run_dfs(agent)
        pseudo_tree_list.append(agent)


    # pseudo-children and pseudo-parents
    for agent in agents:
        for nei in agent.nei_list:
            if nei not in agent.children and agent.level < nei.level:
                agent.pseudo_children.append(nei)
            if nei != agent.parent and agent.level > nei.level:
                agent.pseudo_parents.append(nei)

    # plot
    # plot_edges(randomly_ordered_agents)

    return pseudo_tree_list


def get_graph_list(root: BnBAgent) -> List[BnBAgent]:
    graph_list: List[BnBAgent] = []
    stack: Deque[BnBAgent] = deque()
    stack.append(root)
    while len(stack) > 0:
        next_a = stack.pop()
        graph_list.append(next_a)
        for a in next_a.children:
            stack.append(a)
    return graph_list


def bnb_step(from_agent: BnBAgent, to_agent: BnBAgent, UB, LB, CPA, n_g_agents: int) -> Tuple[bool, BnBAgent, Any, float, dict]:

    # all
    pass

    # root
    if to_agent.parent is None:

        if len(to_agent.bnb_next_n_deque) == 0:
            return True, to_agent, 0, 0, {}

        if from_agent is None or len(CPA) == n_g_agents:
            if len(CPA) == n_g_agents:
                to_agent.next_n_cost_cpa_dict[CPA[to_agent.name]] = (LB, CPA)

            possible_next_n_name = to_agent.bnb_next_n_deque.pop()
            CPA = {to_agent.name: possible_next_n_name}
            LB = to_agent.unary_c[possible_next_n_name]
            UB = 1e7 if len(to_agent.next_n_cost_cpa_dict) == 0 else min([i[0] for i in to_agent.next_n_cost_cpa_dict.values()])
            to_agent.bnb_next_a_deque = deque(to_agent.children)
            next_agent = to_agent.bnb_next_a_deque.pop()
            return False, next_agent, UB, LB, CPA

        # if you here that means the from_agent is some middle child
        if LB >= UB:
            to_agent.next_n_cost_cpa_dict[CPA[to_agent.name]] = (LB, {})
            possible_next_n_name = to_agent.bnb_next_n_deque.pop()
            CPA = {to_agent.name: possible_next_n_name}
            LB = to_agent.unary_c[possible_next_n_name]
            UB = min([i[0] for i in to_agent.next_n_cost_cpa_dict.values()])
            to_agent.bnb_next_a_deque = deque(to_agent.children)
            next_agent = to_agent.bnb_next_a_deque.pop()
            return False, next_agent, UB, LB, CPA

        # if you are here: LB < UB
        next_agent = to_agent.bnb_next_a_deque.pop()
        return False, next_agent, UB, LB, CPA

    # leaf and middle agent
    pass

    # leaf
    if len(to_agent.children) == 0:
        return False, None

    # middle agent
    pass

    return False, None


def execute_branch_and_bound(root: BnBAgent) -> None:
    LB, UB = 0, math.inf
    graph_list = get_graph_list(root)
    n_g_agents = len(graph_list)
    CPA = {}
    finished = False
    from_agent = None
    to_agent = root
    while not finished:
        finished, next_agent, UB, LB, CPA = bnb_step(from_agent, to_agent, UB, LB, CPA, n_g_agents)
        from_agent = to_agent
        to_agent = next_agent

    return


def run_bnb(start_nodes, goal_nodes, nodes, nodes_dict, h_func, **kwargs) -> Tuple[Dict[str, list] | None, dict]:
    """
    :param start_nodes:
    :param goal_nodes:
    :param nodes:
    :param nodes_dict:
    :param h_func:
    :param kwargs:
    - plotter
    - middle_plot
    - map_dim
    - final_plot
    - max_time
    - a_star_iter_limit
    - a_star_calls_limit
    - a_star_closed_nodes_limit
    - alg_name
    - i_run
    - img_dir
    :return:
    """
    max_time = kwargs['max_time']  # seconds

    # preps
    alg_info = get_alg_info_dict()

    # create agents
    agents = []
    for a_index, (s_node, g_node) in enumerate(zip(start_nodes, goal_nodes)):
        agent = BnBAgent(a_index, s_node, g_node, nodes, nodes_dict, h_func)
        agents.append(agent)

    # step iterations
    start_time = time.time()
    step = 0
    while True:
        step += 1

        # Build a pseudo-tree
        pseudo_tree_list = get_pseudo_trees(agents)

        # find the best future move
        for root in pseudo_tree_list:
            execute_branch_and_bound(root)

        # execute the move + check
        pass

        # check if everybody arrived -> return
        pass

        # time check -> return
        runtime = time.time() - start_time
        if runtime > max_time:
            return None, {'agents': agents, 'success_rate': 0}

        # print + plot
        print(f'\r{step=}, {runtime=:.2f}', end='')

    # return None, {'agents': agents, 'success_rate': 0}


def main():
    n_agents = 400
    # img_dir = 'my_map_10_10_room.map'  # 10-10
    # img_dir = 'empty-48-48.map'  # 48-48
    img_dir = 'random-32-32-10.map'  # 32-32
    # img_dir = 'random-64-64-10.map'  # 64-64
    # img_dir = 'warehouse-10-20-10-2-1.map'  # 63-161
    # img_dir = 'lt_gallowstemplar_n.map'  # 180-251
    # img_dir = 'random-32-32-10.map'  # 32-32               | LNS |
    # img_dir = 'ht_chantry.map'  # 162-141   | Up to 230 agents with h=w=30, lim=10sec.

    # random_seed = True
    random_seed = False
    seed = 878
    PLOT_PER = 1
    PLOT_RATE = 0.5

    # --------------------------------------------------- #
    # --------------------------------------------------- #
    # for the algorithms
    alg_name = f'B&B'
    # --------------------------------------------------- #
    # --------------------------------------------------- #

    to_use_profiler = True
    # to_use_profiler = False
    profiler = cProfile.Profile()
    if to_use_profiler:
        profiler.enable()
    for i in range(3):
        print(f'\n[run {i}]')
        result, info = test_mapf_alg_from_pic(
            algorithm=run_bnb,
            img_dir=img_dir,
            alg_name=alg_name,
            n_agents=n_agents,
            random_seed=random_seed,
            seed=seed,
            final_plot=True,
            a_star_iter_limit=5e7,
            # limit_type='norm_time',
            limit_type='dist_time',
            max_time=50,
            a_star_closed_nodes_limit=1e6,
            plot_per=PLOT_PER,
            plot_rate=PLOT_RATE,
        )

        if not random_seed:
            break

        # plt.show()
        plt.close()

    if to_use_profiler:
        profiler.disable()
        stats = pstats.Stats(profiler).sort_stats('cumtime')
        stats.dump_stats('../stats/results_bnb.pstat')


if __name__ == '__main__':
    main()

