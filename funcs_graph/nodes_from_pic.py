from globals import *
from funcs_graph.map_dimensions import map_dimensions_dict
from simulator_objects import Node


def build_graph_from_png(img_png, path='maps', show_map=False):
    print('Start to build_graph_from_png...')
    img_tensor = torchvision.io.read_image(f'{path}/{img_png}', ImageReadMode.GRAY)
    if img_png in map_dimensions_dict:
        img_np = T.Resize(size=map_dimensions_dict[img_png])(img_tensor).squeeze().numpy()
    else:
        img_np = img_tensor.squeeze().numpy()
    max_num = np.max(img_np)
    img_filtered_np = np.where(img_np < max_num, 0, 1)

    return build_graph_from_np(img_filtered_np, show_map)


def distance_nodes(node1, node2, h_func: dict = None):
    if h_func is None:
        # print('regular distance')
        return np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)
    else:
        heuristic_dist = h_func[node1.x][node1.y][node2.x][node2.y]
        # direct_dist = np.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)
        return heuristic_dist


def set_nei(name_1, name_2, nodes_dict):
    if name_1 in nodes_dict and name_2 in nodes_dict and name_1 != name_2:
        node1 = nodes_dict[name_1]
        node2 = nodes_dict[name_2]
        dist = distance_nodes(node1, node2)
        if dist == 1:
            node1.neighbours.append(node2.xy_name)
            node2.neighbours.append(node1.xy_name)


def make_self_neighbour(nodes):
    for node_1 in nodes:
        node_1.neighbours.append(node_1.xy_name)


def make_neighbours(nodes):
    for node_1 in nodes:
        node_1.neighbours.append(node_1.xy_name)
        for node_2 in nodes:
            if node_1.xy_name != node_2.xy_name:
                dist = math.sqrt((node_1.x - node_2.x)**2 + (node_1.y - node_2.y)**2)
                if dist == 1.0:
                    node_1.neighbours.append(node_2.xy_name)


def build_graph_from_np(img_np, show_map=False):
    # 0 - wall, 1 - free space
    nodes = []
    nodes_dict = {}

    x_size, y_size = img_np.shape
    # CREATE NODES
    for i_x in range(x_size):
        for i_y in range(y_size):
            if img_np[i_x, i_y] == 1:
                node = Node(i_x, i_y)
                nodes.append(node)
                nodes_dict[node.xy_name] = node

    # CREATE NEIGHBOURS
    # make_neighbours(nodes)

    name_1, name_2 = '', ''
    for i_x in range(x_size):
        for i_y in range(y_size):
            name_2 = f'{i_x}_{i_y}'
            set_nei(name_1, name_2, nodes_dict)
            name_1 = name_2

    print('finished rows')

    for i_y in range(y_size):
        for i_x in range(x_size):
            name_2 = f'{i_x}_{i_y}'
            set_nei(name_1, name_2, nodes_dict)
            name_1 = name_2
    make_self_neighbour(nodes)
    print('finished columns')

    if show_map:
        plt.imshow(img_np, cmap='gray', origin='lower')
        plt.show()
        # plt.pause(1)
        # plt.close()

    return nodes, nodes_dict


def main():
    nodes, nodes_dict = build_graph_from_png(img_png='22_22_blank_grid_rate_0.1.png', path='../maps', show_map=True)
    print()


if __name__ == '__main__':
    main()


