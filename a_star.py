import matplotlib.pyplot as plt
import numpy as np
from simulator_objects import Node


def heuristic(from_node, to_node):
    return np.sqrt((from_node.x - to_node.x) ** 2 + (from_node.y - to_node.y) ** 2)


def get_lowest_f_node_from(open_list):
    lowest_node = open_list[0]
    for node in open_list:
        if node.f() < lowest_node.f():
            lowest_node = node
    return lowest_node


def get_node(successor_ID, nodes):
    for node in nodes:
        if node.ID == successor_ID:
            return node
    return None


# def way(node_current, node_successor):
#     return heuristic(node_current, node_successor)


def a_star(start, goal, nodes, h_func):
    open_list = []
    close_list = []
    node_current = start
    node_current.h = h_func(start, goal)
    open_list.append(start)
    while len(open_list) > 0:
        node_current = get_lowest_f_node_from(open_list)
        if node_current.ID == goal.ID:
            break
        for successor_ID in node_current.neighbours:
            node_successor = get_node(successor_ID, nodes)
            successor_current_cost = node_current.g + h_func(node_current, node_successor)
            if node_successor in open_list:
                if node_successor.g <= successor_current_cost:
                    continue
            elif node_successor in close_list:
                if node_successor.g <= successor_current_cost:
                    continue
                close_list.remove(node_successor)
                open_list.append(node_successor)
            else:
                open_list.append(node_successor)
                node_successor.h = h_func(node_successor, goal)
            node_successor.g = successor_current_cost
            node_successor.parent = node_current

        open_list.remove(node_current)
        close_list.append(node_current)

    if node_current.ID != goal.ID:
        return None
    else:
        path = []
        while node_current is not None:
            path.append(node_current)
            node_current = node_current.parent
        return path


def main():
    nodes = [
        Node(ID=1, x=1, y=5, neighbours=[3]),
        Node(ID=2, x=3, y=5, neighbours=[3]),
        Node(ID=3, x=3, y=4, neighbours=[1, 2, 4, 6]),
        Node(ID=4, x=1, y=3, neighbours=[3, 5]),
        Node(ID=5, x=3, y=2, neighbours=[4, 6, 7]),
        Node(ID=6, x=4, y=3, neighbours=[3, 5]),
        Node(ID=7, x=2, y=1, neighbours=[5]),
    ]
    node_start = nodes[0]
    node_goal = nodes[-1]

    result = a_star(start=node_start, goal=node_goal, nodes=nodes, h_func=heuristic)

    # PLOT RESULTS:

    # plot field
    x_list = [node.x for node in nodes]
    y_list = [node.y for node in nodes]
    plt.scatter(x_list, y_list)

    # plot found path
    if result is not None:
        parent = result[0]
        successor = parent
        for node in result:
            parent = node
            plt.text(node.x, node.y, f'{node.ID}', bbox={'facecolor': 'yellow', 'alpha': 1, 'pad': 10})
            plt.plot([successor.x, parent.x], [successor.y, parent.y])
            successor = node

    plt.show()


if __name__ == '__main__':
    main()
