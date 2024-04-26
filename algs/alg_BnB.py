import heapq
import math
import random
import time
import concurrent.futures
from functions import *
import cProfile
import pstats
from typing import *
from algs.test_mapf_alg import test_mapf_alg_from_pic
from algs.metrics import c_v_check_for_agent, c_e_check_for_agent, build_constraints, \
    limit_is_crossed, get_agents_in_conf, check_plan, get_alg_info_dict, iteration_print
from algs.metrics import just_check_k_step_plans, just_check_plans
from simulator_objects import Node
from funcs_plotter.plot_functions import plot_step_in_mapf_paths
from create_animation import do_the_animation
from funcs_graph.nodes_from_pic import get_np_from_dot_map


class BnBAgent:
    def __init__(self, index, start_node, goal_node, nodes, nodes_dict, h_func):
        self.index = index
        self.start_node: Node = start_node
        self.curr_node: Node = start_node
        self.goal_node: Node = goal_node
        self.nodes: List[Node] = nodes
        self.nodes_dict: Dict[str, Node] = nodes_dict
        self.h_func = h_func
        self.path = [start_node]

        # stats
        self.stats_runtime = 0
        self.stats_n_messages = 0
        self.stats_n_step_m = 0
        self.stats_n_step_m_list = []  # !!!
        # nei
        self.nei_list: List[Self] = []
        self.nei_dict: Dict[str, Self] = {}
        self.nei_paths_dict = {}
        # bnb
        self.parent: Self = None
        self.pseudo_parents: List[Self] = []
        self.children: List[Self] = []
        self.pseudo_children: List[Self] = []
        self.descendants: List[Self] = []
        self.ancestors: List[Self] = []
        self.parent_cpa: Dict[str, str] = {}
        self.visited: bool = False
        self.level: int = -1
        self.bnb_next_n_deque = deque(self.curr_node.neighbours)
        self.bnb_next_a_deque = deque()
        # self.cost_to_cpa_list: List[Tuple[float, dict]] = []
        self.cost_to_cpa_per_node_dict: Dict[str, Tuple[float, Dict[str, str]]] = {}
        self.next_assignment: Dict[str, str] = {}
        self.curr_next_n = None
        self.unary_c = {}

    def step_reset(self):
        self.nei_list = []
        self.nei_dict = {}
        self.nei_paths_dict = {}
        # stats
        self.stats_n_step_m = 0
        # bnb
        self.parent = None
        self.pseudo_parents: List[Self] = []
        self.children = []
        self.pseudo_children: List[Self] = []
        self.descendants: List[Self] = []
        self.ancestors: List[Self] = []
        self.parent_cpa: Dict[str, str] = {}
        self.visited = False
        self.level = -1
        random.shuffle(self.curr_node.neighbours)
        self.bnb_next_n_deque = deque(self.curr_node.neighbours)
        self.bnb_next_a_deque = deque()
        # self.cost_to_cpa_list = []
        self.cost_to_cpa_per_node_dict: Dict[str, Tuple[float, Dict[str, str]]] = {}
        self.next_assignment: Dict[str, str] = {}
        self.curr_next_n = None
        # update unary table
        self.unary_c = {}
        min_h = min([self.h_func(self.nodes_dict[n_name], self.goal_node) for n_name in self.curr_node.neighbours])
        for nei_node_name in self.curr_node.neighbours:
            nei_node = self.nodes_dict[nei_node_name]
            next_h = self.h_func(nei_node, self.goal_node)
            self.unary_c[nei_node_name] = next_h - min_h

    def add_nei(self, agent: Self):
        self.nei_list.append(agent)
        self.nei_dict[agent.name] = agent

    def get_cut_path(self):
        return cut_back_path(self.path, self.goal_node)

    def __eq__(self, other: Self) -> bool:
        return self.index == other.index

    @property
    def name(self):
        return f'agent_{self.index}'

    @property
    def domain(self):
        return self.curr_node.neighbours

    @property
    def num(self):
        return self.index

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
        # g.attr('edge', color="blue")
        for nei in agent.nei_list:
            # pseudo-child
            if nei not in agent.children and agent.level < nei.level:
                g.edge(agent.name, nei.name, label='PC')

    g.render()


def run_dfs(agent: BnBAgent, parent=None) -> None:
    agent.visited = True
    agent.parent = parent
    if parent:
        agent.level = parent.level + 1
    for nei in agent.nei_list:
        if not nei.visited:
            run_dfs(nei, parent=agent)
            agent.children.append(nei)


def find_descendants(agent: BnBAgent) -> List[BnBAgent]:
    s = deque([agent])
    descendants: List[BnBAgent] = []
    while len(s) > 0:
        next_a = s.pop()
        for child in next_a.children:
            s.append(child)
            descendants.append(child)
    return descendants


def find_ancestors(agent: BnBAgent) -> List[BnBAgent]:
    ancestors: List[BnBAgent] = []
    next_a = agent.parent
    while next_a is not None:
        ancestors.append(next_a)
        next_a = next_a.parent
    return ancestors


def get_pseudo_trees(agents: List[BnBAgent], to_plot_edges: bool = False) -> list:
    pseudo_tree_list = []
    # shuffle
    randomly_ordered_agents: List[BnBAgent] = agents[:]
    random.shuffle(randomly_ordered_agents)

    # create edges
    for agent1, agent2 in combinations(randomly_ordered_agents, 2):
        if len(set(agent1.curr_node.neighbours).intersection(agent2.curr_node.neighbours)) > 0:
            agent1.add_nei(agent2)
            agent2.add_nei(agent1)

    # DFSs
    for agent in randomly_ordered_agents:
        if agent.visited:
            continue
        agent.level = 1
        run_dfs(agent)
        pseudo_tree_list.append(agent)

    # pseudo-children and pseudo-parents + ancestors and descendants
    for agent in agents:
        for nei in agent.nei_list:
            if nei not in agent.children and agent.level < nei.level:
                agent.pseudo_children.append(nei)
            if agent.parent and nei != agent.parent and agent.level > nei.level:
                agent.pseudo_parents.append(nei)
        agent.ancestors = find_ancestors(agent)
        agent.descendants = find_descendants(agent)

    # plot
    if to_plot_edges:
        plot_edges(randomly_ordered_agents)

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


def build_cpa_constraints(CPA: dict, agents_dict: Dict[str, BnBAgent]) -> Tuple[List[str], List[Tuple[str, str]]]:
    c_v_list = []
    c_e_list = []
    heapq.heapify(c_v_list)
    heapq.heapify(c_e_list)
    for cpa_a_name, cpa_next_n in CPA.items():
        heapq.heappush(c_v_list, cpa_next_n)
        # c_v_list.append(cpa_next_n)
        cpa_a = agents_dict[cpa_a_name]
        heapq.heappush(c_e_list, (cpa_next_n, cpa_a.curr_node_name))
        # c_e_list.append((cpa_next_n, cpa_a.curr_node_name))
    return c_v_list, c_e_list


def get_min_n_and_v(curr_node_name, next_n_list, c_v_list, c_e_list, unary_c) -> Tuple[str, float]:
    n_to_v_list = []
    for next_n in next_n_list:
        if next_n in c_v_list:
            n_to_v_list.append((next_n, 1e7))
            continue
        if (curr_node_name, next_n) in c_e_list:
            n_to_v_list.append((next_n, 1e7))
            continue
        n_to_v_list.append((next_n, unary_c[next_n]))
    min_n, min_v = min(n_to_v_list, key=lambda x: x[1])
    return min_n, min_v


def get_cpa_cost(CPA: Dict[str, str], agents_dict: Dict[str, BnBAgent]) -> float:
    cost = 0
    for k, v in CPA.items():
        cost += agents_dict[k].unary_c[v]
    return cost


def del_descendants_and_myself_from_cpa(agent: BnBAgent, CPA: Dict[str, str]):
    if agent.name in CPA:
        del CPA[agent.name]
        for d in agent.descendants:
            if d.name in CPA:
                del CPA[d.name]
    # if agent.parent:
    #     assert agent.parent.name in CPA
    # for an in agent.ancestors:
    #     assert an.name in CPA


def look_ahead(agent: BnBAgent, next_possible_n: str, given_LB: float, given_UB: float) -> float:
    return 0
    if given_LB >= given_UB:
        return 0
    added_costs = 0
    main_curr_node_name = agent.curr_node_name
    # children + pseudo-children
    c_and_pc_list: List[BnBAgent] = [*agent.children, *agent.pseudo_children]
    for cpc_agent in c_and_pc_list:
        cpc_unary_c = {k: v for k, v in cpc_agent.unary_c.items()}
        cpc_curr_node_name = cpc_agent.curr_node_name
        if next_possible_n == cpc_curr_node_name:
            # vc + ec
            del cpc_unary_c[next_possible_n]
            del cpc_unary_c[main_curr_node_name]
        else:
            # vc
            if next_possible_n in cpc_unary_c:
                del cpc_unary_c[next_possible_n]

        if len(cpc_unary_c) == 0:
            return 1e7

        # unary c
        min_v = min(cpc_unary_c.values())
        added_costs += min_v

        if given_LB + added_costs >= given_UB:
            return added_costs

    return added_costs


def pibt_func(agent: BnBAgent, vc_set: List[str], ec_set: List[Tuple[str, str]]) -> bool:
    nei_nodes_names = agent.curr_node.neighbours[:]
    nei_nodes_names.sort(key=lambda n: agent.unary_c[n])
    for nei_node_name in nei_nodes_names:
        # vc
        if nei_node_name in vc_set:
            continue
        if (agent.curr_node_name, nei_node_name) in ec_set:
            continue
        heapq.heappush(vc_set, nei_node_name)
        heapq.heappush(ec_set, (nei_node_name, agent.curr_node_name))
        succeeded = True
        for c in agent.children:
            succeeded = pibt_func(c, vc_set, ec_set)
            if not succeeded:
                break
        if not succeeded:
            vc_set.remove(nei_node_name)
            ec_set.remove((nei_node_name, agent.curr_node_name))
            continue
        agent.pibt_next_n = nei_node_name
        return True
    agent.pibt_next_n = agent.curr_node_name
    return False


def ub_from_pibt(root: BnBAgent, agents_dict: Dict[str, BnBAgent]) -> float:
    pibt_func(root, [], [])
    cpa = {root.name: root.pibt_next_n}
    for d in root.descendants:
        cpa[d.name] = d.pibt_next_n
    UB = get_cpa_cost(cpa, agents_dict)
    return UB


def ub_from_goal_locations(root: BnBAgent):
    init_ub = 0
    if root.curr_node != root.goal_node:
        init_ub += 1
    for d in root.descendants:
        if d.curr_node != root.goal_node:
            init_ub += 1
    return init_ub


def freeze_func(agent, cpa):
    freeze = False
    if agent.name not in cpa:
        freeze = True
    for d in agent.descendants:
        if d.name not in cpa:
            freeze = True
            break
    if freeze:
        cpa[agent.name] = agent.curr_node_name
        for d in agent.descendants:
            cpa[d.name] = d.curr_node_name
    return cpa


def process_step_root(from_agent: BnBAgent, to_agent: BnBAgent, UB, LB, CPA, n_g_agents: int, agents: List[BnBAgent], agents_dict: Dict[str, BnBAgent], step: int, to_assert: bool = False) -> Tuple[bool, BnBAgent, Any, float, dict]:
    # the start
    if from_agent is None:
        init_UB = ub_from_goal_locations(to_agent) + 1
        to_agent.init_ub = init_UB

        while len(to_agent.bnb_next_n_deque) > 0:
            next_possible_n = to_agent.bnb_next_n_deque.pop()
            new_LB = to_agent.unary_c[next_possible_n]
            look_ahead_v = look_ahead(to_agent, next_possible_n, new_LB, init_UB)
            if new_LB + look_ahead_v >= init_UB:
                assert next_possible_n not in to_agent.cost_to_cpa_per_node_dict
                to_agent.cost_to_cpa_per_node_dict[next_possible_n] = (new_LB + look_ahead_v + 100, {})
                continue
            new_CPA = {to_agent.name: next_possible_n}
            if len(to_agent.children) == 0:
                return False, to_agent, init_UB, LB, new_CPA
            to_agent.bnb_next_a_deque = deque(to_agent.children)
            next_agent = to_agent.bnb_next_a_deque.pop()
            return False, next_agent, init_UB, new_LB, new_CPA

    # if you here, it came from the root itself
    if from_agent == to_agent:
        assert len(to_agent.children) == 0
        curr_next_n = CPA[to_agent.name]
        if to_assert:
            assert curr_next_n not in to_agent.cost_to_cpa_per_node_dict
        to_agent.cost_to_cpa_per_node_dict[curr_next_n] = (LB, CPA)
        if len(to_agent.bnb_next_n_deque) == 0:
            min_v, min_cpa = min(to_agent.cost_to_cpa_per_node_dict.values(), key=lambda x: x[0])
            assert min_v < 1e7
            to_agent.next_assignment = min_cpa
            return True, to_agent, 0, 0, {}
        possible_next_n_name = to_agent.bnb_next_n_deque.pop()
        CPA = {to_agent.name: possible_next_n_name}
        LB = to_agent.unary_c[possible_next_n_name]
        return False, to_agent, UB, LB, CPA

    # from a child
    if from_agent in to_agent.children:

        if LB < UB and len(to_agent.bnb_next_a_deque) > 0:
            next_agent = to_agent.bnb_next_a_deque.pop()
            return False, next_agent, UB, LB, CPA

        curr_next_n = CPA[to_agent.name]
        if to_assert:
            assert curr_next_n not in to_agent.cost_to_cpa_per_node_dict
        to_agent.cost_to_cpa_per_node_dict[curr_next_n] = (LB, CPA)
        UB = min([x[0] for x in to_agent.cost_to_cpa_per_node_dict.values()])
        UB = min(UB, to_agent.init_ub + 1)

        while len(to_agent.bnb_next_n_deque) > 0:
            next_possible_n = to_agent.bnb_next_n_deque.pop()
            new_LB = to_agent.unary_c[next_possible_n]
            # look_ahead()
            look_ahead_v = look_ahead(to_agent, next_possible_n, new_LB, UB)
            if new_LB + look_ahead_v >= UB:
                assert next_possible_n not in to_agent.cost_to_cpa_per_node_dict
                to_agent.cost_to_cpa_per_node_dict[next_possible_n] = (new_LB + look_ahead_v + 100, {})
                continue
            new_CPA = {to_agent.name: next_possible_n}
            to_agent.bnb_next_a_deque = deque(to_agent.children)
            next_agent = to_agent.bnb_next_a_deque.pop()
            return False, next_agent, UB, new_LB, new_CPA

        min_v, min_cpa = min(to_agent.cost_to_cpa_per_node_dict.values(), key=lambda x: x[0])
        min_cpa = freeze_func(to_agent, min_cpa)
        to_agent.next_assignment = min_cpa
        if to_assert:
            assert min_v < 1e7
            assert to_agent.name in min_cpa
            for d in to_agent.descendants:
                assert d.name in min_cpa
        return True, to_agent, 0, 0, {}

    raise RuntimeError('not here')


def no_move_cost_check(min_n: str, v: float, CPA: dict, to_agent: BnBAgent) -> None:
    for an in to_agent.ancestors:
        if CPA[an.name] != an.curr_node_name:
            return
    if min_n:
        if min_n != to_agent.curr_node_name:
            return
    assert v < 1e7


def process_step_leaf(from_agent: BnBAgent, to_agent: BnBAgent, UB, LB, CPA, n_g_agents: int, agents: List[BnBAgent], agents_dict: Dict[str, BnBAgent], step: int, to_assert: bool = False) -> Tuple[bool, BnBAgent, Any, float, dict]:
    if to_assert:
        assert to_agent.name not in CPA
        assert to_agent.parent == from_agent
        for nei in to_agent.nei_list:
            assert nei.name in CPA
    c_v_list, c_e_list = build_cpa_constraints(CPA, agents_dict)
    min_n, min_v = get_min_n_and_v(to_agent.curr_node_name, to_agent.curr_node.neighbours, c_v_list, c_e_list, to_agent.unary_c)
    if LB + min_v >= UB:
        return False, to_agent.parent, UB, LB + min_v + 100, CPA
    CPA[to_agent.name] = min_n
    LB += to_agent.unary_c[min_n]
    return False, to_agent.parent, UB, LB, CPA


def all_descendants_in_cpa(agent: BnBAgent, CPA: Dict[str, str]) -> bool:
    for desc in agent.descendants:
        if desc.name not in CPA:
            return False
    return True


def process_step_middle_agent(from_agent: BnBAgent, to_agent: BnBAgent, UB, LB, CPA, n_g_agents: int, agents: List[BnBAgent], agents_dict: Dict[str, BnBAgent], step: int, to_assert: bool = False) -> Tuple[bool, BnBAgent, Any, float, dict]:
    # from parent
    if from_agent == to_agent.parent:
        if to_assert:
            assert LB < UB
            assert to_agent.name not in CPA
            for d in to_agent.descendants:
                assert d.name not in CPA
        to_agent.parent_cpa = {k: v for k, v in CPA.items()}
        to_agent.cost_to_cpa_per_node_dict = {}  # Dict[str, Tuple[float, Dict[str, str]]]
        to_agent.bnb_next_n_deque = deque(to_agent.curr_node.neighbours)
        random.shuffle(to_agent.bnb_next_n_deque)

        c_v_list, c_e_list = build_cpa_constraints(to_agent.parent_cpa, agents_dict)
        while len(to_agent.bnb_next_n_deque) > 0:
            next_possible_n = to_agent.bnb_next_n_deque.pop()
            if to_assert:
                assert next_possible_n not in to_agent.cost_to_cpa_per_node_dict
            if next_possible_n in c_v_list or (to_agent.curr_node_name, next_possible_n) in c_e_list:
                to_agent.cost_to_cpa_per_node_dict[next_possible_n] = (1e7, to_agent.parent_cpa)
                continue
            new_LB = LB + to_agent.unary_c[next_possible_n]
            # look_ahead:
            look_ahead_v = look_ahead(to_agent, next_possible_n, new_LB, UB)
            if new_LB + look_ahead_v >= UB:
                to_agent.cost_to_cpa_per_node_dict[next_possible_n] = (new_LB + look_ahead_v + 100, to_agent.parent_cpa)
                continue
            new_CPA = {k: v for k, v in to_agent.parent_cpa.items()}
            new_CPA[to_agent.name] = next_possible_n
            LB = get_cpa_cost(new_CPA, agents_dict)
            assert LB < UB
            to_agent.bnb_next_a_deque = deque(to_agent.children)
            next_agent = to_agent.bnb_next_a_deque.pop()
            return False, next_agent, UB, LB, new_CPA

        min_v, min_cpa = min(to_agent.cost_to_cpa_per_node_dict.values(), key=lambda x: x[0])
        assert min_v >= UB
        return False, to_agent.parent, UB, min_v, min_cpa

    # from child
    if from_agent in to_agent.children:
        if LB < UB and len(to_agent.bnb_next_a_deque) > 0:
            next_agent = to_agent.bnb_next_a_deque.pop()
            return False, next_agent, UB, LB, CPA
        curr_next_n = CPA[to_agent.name]
        if to_assert:
            assert curr_next_n not in to_agent.cost_to_cpa_per_node_dict
        to_agent.cost_to_cpa_per_node_dict[curr_next_n] = (LB, CPA)

        c_v_list, c_e_list = build_cpa_constraints(to_agent.parent_cpa, agents_dict)
        while len(to_agent.bnb_next_n_deque) > 0:
            next_possible_n = to_agent.bnb_next_n_deque.pop()
            if to_assert:
                assert next_possible_n not in to_agent.cost_to_cpa_per_node_dict
            if next_possible_n in c_v_list or (to_agent.curr_node_name, next_possible_n) in c_e_list:
                to_agent.cost_to_cpa_per_node_dict[next_possible_n] = (1e7, to_agent.parent_cpa)
                continue
            new_LB = LB + to_agent.unary_c[next_possible_n]
            # look_ahead()
            look_ahead_v = look_ahead(to_agent, next_possible_n, new_LB, UB)
            if new_LB + look_ahead_v >= UB:
                to_agent.cost_to_cpa_per_node_dict[next_possible_n] = (new_LB + look_ahead_v, to_agent.parent_cpa)
                continue
            new_CPA = {k: v for k, v in to_agent.parent_cpa.items()}
            new_CPA[to_agent.name] = next_possible_n
            LB = get_cpa_cost(new_CPA, agents_dict)
            assert LB < UB
            to_agent.bnb_next_a_deque = deque(to_agent.children)
            next_agent = to_agent.bnb_next_a_deque.pop()
            return False, next_agent, UB, LB, new_CPA

        min_v, min_cpa = min(to_agent.cost_to_cpa_per_node_dict.values(), key=lambda x: x[0])
        if min_v < UB:
            if to_assert:
                assert to_agent.name in min_cpa
                for d in to_agent.descendants:
                    assert d.name in min_cpa
        return False, to_agent.parent, UB, min_v, min_cpa

    raise RuntimeError('nope')


def bnb_step(from_agent: BnBAgent, to_agent: BnBAgent, UB, LB, CPA, n_g_agents: int, agents: List[BnBAgent], agents_dict: Dict[str, BnBAgent], step: int, to_assert: bool = False) -> Tuple[bool, BnBAgent, Any, float, dict]:

    to_agent.stats_n_step_m += 1
    # ------------------------- root -------------------------
    if to_agent.parent is None:
        return process_step_root(from_agent, to_agent, UB, LB, CPA, n_g_agents, agents, agents_dict, step, to_assert)

    # ------------------------- middle agent -------------------------
    if to_agent.parent is not None and len(to_agent.children) > 0:
        return process_step_middle_agent(from_agent, to_agent, UB, LB, CPA, n_g_agents, agents, agents_dict, step, to_assert)

    # ------------------------- leaf -------------------------
    if len(to_agent.children) == 0:
        return process_step_leaf(from_agent, to_agent, UB, LB, CPA, n_g_agents, agents, agents_dict, step, to_assert)

    raise RuntimeError('noooo')


def execute_branch_and_bound(
        root: BnBAgent,
        agents: List[BnBAgent], agents_dict: Dict[str, BnBAgent], step: int, to_assert: bool = False,
        runtime: float = 0, start_step_time: float = 0, max_time: float = 1e7
) -> None:
    LB, UB = 0, math.inf
    graph_list = get_graph_list(root)
    n_g_agents = len(graph_list)
    CPA = {}
    finished = False
    from_agent = None
    to_agent = root
    visited_list = []
    # print()
    while not finished:
        finished, next_agent, UB, LB, CPA = bnb_step(from_agent, to_agent, UB, LB, CPA, n_g_agents, agents, agents_dict, step, to_assert)
        visited_list.append((to_agent.name, LB, UB))
        from_agent = to_agent
        to_agent = next_agent
        print(f'\r[{n_g_agents}][{5**n_g_agents}][{root.name}][{UB=}][LB={int(LB)}][{len(CPA)}/{n_g_agents}][level={to_agent.level}]{len(visited_list)=}', end='')
        if runtime + (time.time() - start_step_time) > max_time:
            root.next_assignment = freeze_func(root, root.next_assignment)
            return
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
    to_render = kwargs['to_render']
    plot_rate = kwargs['plot_rate']
    map_dim = kwargs['map_dim']
    img_dir = kwargs['img_dir']
    to_plot_edges = kwargs['to_plot_edges']
    to_assert = kwargs['to_assert']
    if to_render:
        fig, ax = plt.subplots(1, 2, figsize=(14, 7))

    max_time = kwargs['my_max_time'] if 'my_max_time' in kwargs else kwargs['max_time']  # minutes
    max_time = max_time * 60  # seconds

    # preps
    alg_info = get_alg_info_dict()

    # create agents
    agents, agents_dict = [], {}
    for a_index, (s_node, g_node) in enumerate(zip(start_nodes, goal_nodes)):
        agent = BnBAgent(a_index, s_node, g_node, nodes, nodes_dict, h_func)
        agents.append(agent)
        agents_dict[agent.name] = agent

    # step iterations
    runtime = 0
    step = 0
    while True:
        step += 1

        # reset
        for agent in agents:
            agent.step_reset()

        # Build a pseudo-tree
        pseudo_tree_list: List[BnBAgent] = get_pseudo_trees(agents, to_plot_edges)

        # find the best future move
        # -------------------------------------------------------------------------------------- #
        step_time_list = []
        for root in pseudo_tree_list:
            start_step_time = time.time()
            execute_branch_and_bound(root, agents, agents_dict, step, to_assert=to_assert, runtime=runtime, start_step_time=start_step_time, max_time=max_time)
            end_step_time = time.time()
            step_time_list.append(end_step_time - start_step_time)
        runtime += max(step_time_list)
        # -------------------------------------------------------------------------------------- #
        # with concurrent.futures.ThreadPoolExecutor(max_workers=len(pseudo_tree_list)) as executor:
        #     for i, root in enumerate(pseudo_tree_list):
        #         executor.submit(execute_branch_and_bound, root, agents, agents_dict, step)
        # -------------------------------------------------------------------------------------- #

        # stats
        for agent in agents:
            agent.stats_n_step_m_list.append(agent.stats_n_step_m)

        # execute the move + check
        l = set([len(a.path) for a in agents])
        assert len(set([len(a.path) for a in agents])) == 1
        for root in pseudo_tree_list:
            # r_name = root.name
            # r_cost_to_cpa_per_node_dict = root.cost_to_cpa_per_node_dict
            # r_domain = root.domain
            # r_curr_node_name = root.curr_node_name
            # r_next_node_name = r_next_assignment[root.name]
            # print('', end='')
            r_next_assignment = root.next_assignment
            for a_name, n_name in r_next_assignment.items():
                i_agent = agents_dict[a_name]
                new_curr_node = nodes_dict[n_name]
                i_agent.path.append(new_curr_node)
                i_agent.curr_node = new_curr_node

        # check if everybody arrived -> return
        finished = True
        for agent in agents:
            if agent.path[-1] != agent.goal_node:
                finished = False
                break
        if finished:
            print(f'#########################################################')
            print(f'#########################################################')
            print(f'#########################################################')
            result = {agent.name: agent.get_cut_path() for agent in agents}
            alg_info['success_rate'] = 1
            cost = sum([len(p) for p in result.values()])
            alg_info['sol_quality'] = cost
            alg_info['runtime'] = runtime
            alg_info['dist_runtime'] = runtime
            # alg_info['n_messages'] =
            alg_info['m_per_step'] = np.sum([np.mean(agent.stats_n_step_m_list) for agent in agents])

            # animate
            # img_np, _ = get_np_from_dot_map(img_dir, path='../maps', )
            # do_the_animation(info={'img_np': img_np, 'agents': agents, 'max_time': step, 'img_dir': img_dir, 'alg_name': 'B&B'}, to_save=False)
            return result, alg_info

        # time check -> return
        if runtime > max_time:
            return None, {'agents': agents, 'success_rate': 0}

        # print + plot
        print(f'\r{step=}, {runtime=:.2f}', end='')
        if to_render:
            plot_info = {'i': step,
                         'iterations': max_time,
                         'paths_dict': {agent.name: agent.path for agent in agents},
                         'nodes': nodes,
                         'side_x': map_dim[0],
                         'side_y': map_dim[1],
                         't': step,
                         'img_dir': img_dir,
                         'i_agent': agents[0],
                         }
            plot_step_in_mapf_paths(ax[0], plot_info)
            plt.pause(plot_rate)


    # return None, {'agents': agents, 'success_rate': 0}


def main():
    n_agents = 100
    # img_dir = 'my_map_10_10_room.map'  # 10-10
    # img_dir = '10_10_my_rand.map'  # 10-10
    # img_dir = 'random-32-32-10.map'  # 32-32
    # img_dir = 'empty-32-32.map'  # 32-32
    img_dir = 'empty-48-48.map'  # 48-48
    # img_dir = 'random-64-64-10.map'  # 64-64
    # img_dir = 'warehouse-10-20-10-2-1.map'  # 63-161
    # img_dir = 'lt_gallowstemplar_n.map'  # 180-251
    # img_dir = 'random-32-32-10.map'  # 32-32               | LNS |
    # img_dir = 'ht_chantry.map'  # 162-141   | Up to 230 agents with h=w=30, lim=10sec.

    # random_seed = True
    random_seed = False
    seed = 878
    PLOT_PER = 1
    plot_rate = 0.01
    to_render = True
    # to_render = False
    set_seed(random_seed_bool=random_seed, seed=seed)

    # --------------------------------------------------- #
    # --------------------------------------------------- #
    # for the algorithms
    alg_name = f'PT-FB-PF'
    # to_plot_edges = True
    to_plot_edges = False
    # to_assert = True
    to_assert = False
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
            max_time=1,
            a_star_closed_nodes_limit=1e6,
            plot_per=PLOT_PER,
            plot_rate=plot_rate,
            to_render=to_render,
            to_plot_edges=to_plot_edges,
            to_assert=to_assert,
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

