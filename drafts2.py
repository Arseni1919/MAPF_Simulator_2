# from a child
if from_agent is not None and from_agent in to_agent.children:
    if LB >= UB or len(CPA) == n_g_agents:
        update_next_cost_to_cpa_list(LB, UB, CPA, to_agent, n_g_agents)
        # if LB >= UB:
        #     to_agent.cost_to_cpa_list.append((1e7, {}))
        # if LB < UB and len(CPA) == n_g_agents:
        #     to_agent.cost_to_cpa_list.append((LB, CPA))
        if len(to_agent.bnb_next_n_deque) == 0:
            return True, to_agent, 0, 0, {}
        possible_next_n_name = to_agent.bnb_next_n_deque.pop()
        CPA = {to_agent.name: possible_next_n_name}
        LB = get_cpa_cost(CPA, agents_dict)
        UB = min([i[0] for i in to_agent.cost_to_cpa_list])
        # if to_agent.num == 70 and to_agent.curr_node_name == '18_29' and step == 4:
        #     if to_agent.name in CPA and CPA[to_agent.name] == '18_29':
        #         print('', end='')
        to_agent.bnb_next_a_deque = deque(to_agent.children)
        next_agent = to_agent.bnb_next_a_deque.pop()
        return False, next_agent, UB, LB, CPA

    # if you are here: LB < UB
    next_agent = to_agent.bnb_next_a_deque.pop()
    return False, next_agent, UB, LB, CPA