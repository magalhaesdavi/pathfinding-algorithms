import random
import numpy as np
import math
from itertools import combinations


def calculate_dist(cords1, cords2):
    return math.sqrt((cords1[0] - cords2[0])**2 + (cords1[1] - cords2[1])**2)


def map_generator(available_nodes, density=0.5, weights_range=(0, 100)):
    """
    Gera uma lista de conexoes com pesos no intervalo 'weights_range'
    baseado numa lista de ids em 'available_nodes', a variavel 'density'
    especifica o quao conectados os nos vao estar.
    """
    map_data = set({})

    rand_cords = list(np.random.randint(weights_range[0], weights_range[1]+1, (len(available_nodes), 2)))
    random.shuffle(available_nodes)

    map_cords = list(map(lambda node_id, cords: (node_id, cords), available_nodes, rand_cords))

    connections = list(combinations(map_cords, 2))
    # random.shuffle(connections)

    # print(map_cords)
    print("")

    for map_cord in map_cords:
        node_edges = list(filter(lambda connection: connection[0][0] == map_cord[0], connections))
        node_edges.sort(key=lambda  connection: calculate_dist(map_cord[1], connection[1][1]))

        print(map_cord)
        print(node_edges[:5])


        size = int(len(node_edges) * density)
        if size > len(node_edges):
            size = len(node_edges)

        node_edges = node_edges[: size]

        for connection in node_edges:

            conn_dist = calculate_dist(connection[0][1], connection[1][1])
            noise = random.random() + 1

            map_data.add(
                (
                    (connection[0][0], tuple(connection[0][1])),
                    (connection[1][0], tuple(connection[1][1])),
                    round(conn_dist * noise, 3)
                )
            )

    print(f"Mapa gerado com {len(map_data)} conexoes")
    return map_data


def find_smaller(d):
    # return min(d, key=d.get)
    # return min(d, key = lambda t: t[2])
    return min(d.items(), key=lambda k: k[1][2])[0]


def heuristic(vertex_a, vertex_b):
    return calculate_dist((vertex_a.vertex_x, vertex_a.vertex_y), (vertex_b.vertex_x, vertex_b.vertex_y))
