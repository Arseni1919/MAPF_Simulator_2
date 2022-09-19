# import matplotlib.pyplot as plt
#
# n = 10
# l = plt.cm.get_cmap('hsv', n)
# for i in range(n):
#     item = l(i)
#     print(item)
import random
import time

l = [0, 1, 2, 3]
l.insert(3, 100)
random.shuffle(l)
print(l)
print(l.index(100))

s = time.time()
time.sleep(1)
elapsed = time.time() - s
print(f"Executed in {elapsed:0.2f} seconds.")








# for constr in c_v_list:
    #     agent_1, agent_2, x, y, t = constr
    #     reg_constr_dict[f'{x}_{y}'].append(t)
    # for constr in c_e_list:
    #     agent_1, agent_2, prev_x, prev_y, x, y, t = constr
    #     reg_constr_dict[f'{x}_{y}'].append(t)
    #     reg_constr_dict[f'{prev_x}_{prev_y}'].append(t)

    # for constr in pbs_node.constraints[update_agent.name]:
    #     x, y, t = constr
    #     reg_constr_dict[f'{x}_{y}'].append(t)


# @preprint_func_name
# def add_new_constraint(pbs_node, index, conf, conf_type):
#     print('\rFUNC: add_new_constraint', end='')
#     agent_name = conf[index]
#     if conf_type == 'vertex':
#         agent_1, agent_2, x, y, t = conf
#         pbs_node.constraints[agent_name].append((x, y, t))
#     elif conf_type == 'edge':
#         agent_1, agent_2, prev_x, prev_y, x, y, t = conf
#         pbs_node.constraints[agent_name].append((x, y, t))
#         pbs_node.constraints[agent_name].append((prev_x, prev_y, t))
#     else:
#         raise RuntimeError('type error')


# def create_other_agents_orders(pbs_node, h_agent: str, l_agent: str):
#
#     if type(h_agent) is not str or type(l_agent) is not str:
#         raise RuntimeError('type(h_agent) is not str or type(l_agent) is not str')
#
#     higher_agents_orders_to_add = []
#     lower_agents_orders_to_add = []
#     for (h_order_agent, l_order_agent) in pbs_node.ordering_rules:
#
#         if l_order_agent == h_agent:
#
#             if (l_agent, h_order_agent) in pbs_node.ordering_rules:
#                 raise RuntimeError('(l_agent, h_order_agent) in pbs_node.ordering_rules')
#
#             higher_agents_orders_to_add.append((h_order_agent, l_agent))
#
#         if h_order_agent == l_agent:
#
#             if (l_order_agent, h_agent) in pbs_node.ordering_rules:
#                 raise RuntimeError('(l_order_agent, h_agent) in pbs_node.ordering_rules')
#
#             lower_agents_orders_to_add.append((h_agent, l_order_agent))
#
#     return higher_agents_orders_to_add, lower_agents_orders_to_add


# def add_up(pbs_node, h_agent: str, l_agent: str):
#     list_to_check = [h_agent]
#     output_list = []
#     iteration = 0
#     while len(list_to_check) != 0:
#         to_check = list_to_check.pop()
#         iteration += 1
#         h_as_lower = [order[0] for order in pbs_node.ordering_rules if order[1] == to_check]
#         for new_one in h_as_lower:
#             if new_one not in list_to_check:
#                 list_to_check.append(new_one)
#                 if (new_one, l_agent) == ('agent_14', 'agent_4'):
#                     print('')
#                 output_list.append((new_one, l_agent))
#
#                 if (l_agent, new_one) in pbs_node.ordering_rules:
#                     raise RuntimeError('(l_agent, new_one) in pbs_node.ordering_rules')
#
#     return output_list
#
#
# def add_down(pbs_node, h_agent: str, l_agent: str):
#     list_to_check = [l_agent]
#     output_list = []
#     iteration = 0
#     while len(list_to_check) != 0:
#         iteration += 1
#         to_check = list_to_check.pop()
#         l_as_higher = [order[1] for order in pbs_node.ordering_rules if order[0] == to_check]
#         for new_one in l_as_higher:
#             if new_one not in list_to_check:
#                 list_to_check.append(new_one)
#                 output_list.append((h_agent, new_one))
#
#                 if (new_one, h_agent) in pbs_node.ordering_rules:
#                     raise RuntimeError('(new_one, h_agent) in pbs_node.ordering_rules')
#
#     return output_list


    # NEW_pbs_node.ordering_rules.append(curr_tuple)
    # higher_agents_orders_to_add = add_up(NEW_pbs_node, curr_tuple[0], curr_tuple[1])
    # NEW_pbs_node.ordering_rules.extend(higher_agents_orders_to_add)
    # lower_agents_orders_to_add = add_down(NEW_pbs_node, curr_tuple[0], curr_tuple[1])
    # NEW_pbs_node.ordering_rules.extend(lower_agents_orders_to_add)