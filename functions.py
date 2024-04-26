from globals import *


def manhattan_distance_nodes(node1, node2):
    return abs(node1.x-node2.x) + abs(node1.y-node2.y)


def rename_nodes_in_path(path):
    for t, node in enumerate(path):
        node.t = t
        node.ID = f'{node.x}_{node.y}_{t}'


def get_color(i):
    index_to_pick = i % len(color_names)
    return color_names[index_to_pick]


def set_seed(random_seed_bool, seed=1):
    if random_seed_bool:
        seed = random.randint(0, 10000)
    random.seed(seed)
    np.random.seed(seed)
    print(f'[SEED]: --- {seed} ---')


def cut_back_path(path, goal_node):
    len_path = len(path)
    if len_path > 1:
        if path[-1].xy_name == goal_node.xy_name:
            for backwards_node in path[:-1][::-1]:
                if backwards_node.xy_name == goal_node.xy_name:
                    len_path -= 1
                else:
                    break
    cut_path = path[:len_path]
    return cut_path
