# from a child
    if from_agent in to_agent.children:

        if LB < UB and len(to_agent.bnb_next_a_deque) > 0:
            next_agent = to_agent.bnb_next_a_deque.pop()
            return False, next_agent, UB, LB, CPA

        for d in to_agent.descendants:
            assert d.name in CPA
        curr_next_n = CPA[to_agent.name]
        assert curr_next_n not in to_agent.cost_to_cpa_per_node_dict
        to_agent.cost_to_cpa_per_node_dict[curr_next_n] = (LB, CPA)
        if len(to_agent.bnb_next_n_deque) == 0:
            min_v, min_cpa = min(to_agent.cost_to_cpa_per_node_dict.values(), key=lambda x: x[0])
            assert min_v < 1e7
            assert to_agent.name in min_cpa
            for d in to_agent.descendants:
                assert d.name in min_cpa
            to_agent.next_assignment = min_cpa
            return True, to_agent, 0, 0, {}
        possible_next_n_name = to_agent.bnb_next_n_deque.pop()
        CPA = {to_agent.name: possible_next_n_name}
        LB = get_cpa_cost(CPA, agents_dict)
        UB = min([i[0] for i in to_agent.cost_to_cpa_per_node_dict.values()])
        to_agent.bnb_next_a_deque = deque(to_agent.children)
        next_agent = to_agent.bnb_next_a_deque.pop()
        return False, next_agent, UB, LB, CPA
