from graph import Graph
from utils import map_generator, calculate_dist
from queue import Queue, PriorityQueue
from collections import defaultdict, deque, namedtuple
import random
import string
import time
import timeit
import math


def irrevocabile(graph, start_id, end_id):

    start_node = [ node for node in list(graph.graph.keys()) if node.vertex_id == start_id ][0]
    terminal_node = [ node for node in list(graph.graph.keys()) if node.vertex_id == end_id ][0]

    # Inicializando flag de sucesso e fracasso
    success = False
    fail = False
    solution = [start_node]
    visited = [start_node]

    while not success or not fail:
        ok = False
        current_node = solution[-1]
        edges = list(graph[current_node].keys())

        while len(edges) > 0:

            # Nesta implementacao a escolha da regra/edge é por ordem alfabética ou crescente (se forem numeros)
            edges.sort()
            chsn_node = edges[0]

            if chsn_node not in visited:
                visited.append(chsn_node)
                if chsn_node not in solution:
                    ok = True
                    break
            else:
                del edges[0]

        if not ok:
            fail = True
            break

        solution.append(chsn_node)
        if chsn_node.vertex_id == end_id:
            success = True
            break

    return [ node.vertex_id for node in solution ], "SUCCESS" if success else "FAILURE"


def backTracking(graph, start_id, end_id):


    start_node = [ node for node in list(graph.graph.keys()) if node.vertex_id == start_id ][0]
    terminal_node = [ node for node in list(graph.graph.keys()) if node.vertex_id == end_id ][0]

    # Inicializando flag de sucesso e fracasso
    success = False
    fail = False
    # A solucao sera descoberta manipulando uma "pilha"
    solution = [start_node]
    visited = [start_node]

    while not success or not fail:
        # Verificamos se e possivel aplicar regras no ultimo no na pilha
        current_node = solution[-1]
        edges = list(G[current_node].keys())

        # Se ainda existe pelo menos uma aresta que ainda não foi visitada entramos no if
        if not all(node in visited for node in edges):

            # Nesta implementacao a escolha da regra/edge é por ordem alfabética ou crescente (se forem numeros)
            edges.sort()
            chsn_node = edges[0]
            while chsn_node in visited:
                chsn_node = edges[edges.index(chsn_node) + 1]

            if chsn_node not in visited:
                visited.append(chsn_node)

            if chsn_node not in solution:
                solution.append(chsn_node)
                # Se o proximo no for o no terminal, ativamos a flag de sucesso e terminamos o loop
                if chsn_node.vertex_id == end_id:
                    success = True
                    break
        else:
            # Se voltarmos para o no inicial, significa que nao ha mais nos para serem explorados
            if current_node.vertex_id == start_id:
                fail = True
                break
            else:
                # Senao, voltamos na "arvore" de busca
                solution.pop()

    return [ node.vertex_id for node in solution ], "SUCCESS" if success else "FAILURE"

def breadth_first_search(graph, start_id, end_id):

    start_node = [ node for node in list(graph.graph.keys()) if node.vertex_id == start_id ][0]
    end_node = [ node for node in list(graph.graph.keys()) if node.vertex_id == end_id ][0]

    parentMap = {}
    visited = []
    solution = []
    current = start_node
    queue = deque()
    queue.append(current)
    visited.append(current)
    success = False
        
    while queue and not success:
        current = queue.popleft()   
        if current == end_node:
            success = True
            break
        else:
            for child in graph[current]:
                if child not in visited:
                    queue.append(child)
                    visited.append(child)
                    parentMap[child.vertex_id] = current.vertex_id
        
    curr_id = current.vertex_id
    if success:
        while curr_id != start_node.vertex_id:
            solution.append(curr_id)
            curr_id = parentMap[curr_id]
        solution.append(curr_id)
        solution.reverse()
        return solution, "success"
    else:
        return solution, "failure"

def depth_first_search(graph, start_id, end_id):

    start_node = [ node for node in list(graph.graph.keys()) if node.vertex_id == start_id ][0]
    end_node = [ node for node in list(graph.graph.keys()) if node.vertex_id == end_id ][0]

    parentMap = {}
    visited = []
    stack = []
    solution = []
    current = start_node
    stack.append(current)
    success = False

    while stack:
        current = stack[-1]
        stack.pop()

        if current not in visited:
            visited.append(current)

        if current == end_node:
            success = True
            break

        for child in graph[current]:
            if child not in visited:
                stack.append(child)
                parentMap[child.vertex_id] = current.vertex_id

    curr_id = current.vertex_id
    if success:
        while curr_id != start_node.vertex_id:
            solution.append(curr_id)
            curr_id = parentMap[curr_id]
        solution.append(curr_id)
        solution.reverse()
        return solution, "success"
    else:
        return solution, "failure"

def ordered_search(graph, start_id, end_id):
    start_node = [ node for node in list(graph.graph.keys()) if node.vertex_id == start_id ][0]
    terminal_node = [ node for node in list(graph.graph.keys()) if node.vertex_id == end_id ][0]

    success = False
    fail = False

    solution = []
    visited = []

    prority_queue = PriorityQueue()
    path_to_goal = []
    prority_queue.put((0, start_node, path_to_goal))

    cost = 0

    while not prority_queue.empty():
        top_q = prority_queue.get()
        current_node = top_q[1]
        current_path = top_q[2]

        # print([node.vertex_id for node in visited])
        # print(current_node.vertex_id)
        # print("")

        if current_node.vertex_id == end_id:
            success = True
            solution = current_path
        else:
            if not current_node in visited:
                edges = list(graph[current_node].keys())
                edges.sort(key=lambda  edge: cost + graph[current_node][edge].weight)
                cost += graph[current_node][edges[0]].weight

                while len(edges) > 0:
                    edge = edges.pop()
                    prority_queue.put((cost + graph[current_node][edge].weight, edge, current_path + [current_node]))

                visited.append(current_node)

    return [ node.vertex_id for node in solution + [terminal_node] ], "SUCCESS" if success else "FAILURE"

def a_star(graph, start, destination):
    pass

if __name__ == "__main__":
    G = Graph()

    nodes_list = list(string.ascii_uppercase)
    test_map = map_generator(nodes_list, 0.1)

    vertex = namedtuple("Vertex", ["vertex_id", "vertex_x", "vertex_y"])
    for connection in test_map:
        node1 = vertex(vertex_id=connection[0][0], vertex_x=connection[0][1][0], vertex_y=connection[0][1][1])
        node2 = vertex(vertex_id=connection[1][0], vertex_x=connection[1][1][0], vertex_y=connection[1][1][1])
        weight = connection[2]
        G.add_edge(node1, node2, weight)

    print(G)

    print(irrevocabile(G, 'B', 'Z'))
    print(backTracking(G, 'B', 'Z'))
    print(depth_first_search(G, 'B', 'Z'))
    print(ordered_search(G, 'B', 'Z'))
    
