import os
import utils
from faker import Faker
from collections import namedtuple, OrderedDict
from graph import Graph
from searchAlgorithms import *


fake = Faker()

tests = [(25, 50, 0.2), (50, 500, 0.08), (100, 1000, 0.05), (200, 1500, 0.045)]

for test in tests:

    for j in range(10):
        G = Graph()
        cities = []

        for i in range(test[0]):
            cities.append(fake.city())

        test_map, most_far_nodes = utils.map_generator(cities, test[2], weights_range=(-1 * test[1], test[1]))
        vertex = namedtuple("Vertex", ["vertex_id", "vertex_x", "vertex_y"])
        for connection in test_map:
            node1 = vertex(vertex_id=connection[0][0], vertex_x=connection[0][1][0], vertex_y=connection[0][1][1])
            node2 = vertex(vertex_id=connection[1][0], vertex_x=connection[1][1][0], vertex_y=connection[1][1][1])
            weight = connection[2]
            G.add_edge(node1, node2, weight)

        file_name = 'graph_n' + str(test[0])
        utils.display_graph(G, file_name)

        algorithms = {
            "Backtracking": backTracking,
            "BFS": breadth_first_search,
            "DFS": depth_first_search,
            "UCS": uniform_cost_search,
            "Greedy": greedy,
            "Astar": a_star,
            "IDAstar": ida_star
        }

        algorithms = OrderedDict(algorithms)
        algo_list = list(algorithms.keys())

        for i, algo_name in enumerate(algo_list):

            header = False
            close = False

            if os.stat("./outputs/results.csv").st_size == 0:
                header = True
            if i == len(algo_list) - 1:
                close = True

            solution, depth, cost, expanded, visited, average, exec_time, result = algorithms[algo_name](G, most_far_nodes[0], most_far_nodes[1])

            utils.save_metrics(
                "results.csv",
                close_on_end=close,
                write_header=header,
                algorithm=algo_name,
                solution=utils.format_solution(solution),
                depth=depth,
                cost=cost,
                expanded_nodes=expanded, 
                visited_nodes=visited, 
                average_branching_factor=average, 
                execution_time=exec_time, 
                result=result,
                n = test[0])
