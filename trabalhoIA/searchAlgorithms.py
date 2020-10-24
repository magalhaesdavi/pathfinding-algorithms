from graph import Graph
from utils import map_Generator
from queue import Queue
from collections import defaultdict, deque, namedtuple
import random
import string


def irrevocabile(graph, start_node_id, terminal_node_id):
    # Inicializando flag de sucesso e fracasso
    success = False
    fail = False
    solution = [start_node_id]
    visited = [start_node_id]

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
        if chsn_node == terminal_node_id:
            success = True
            break

    return solution, "SUCCESS" if success else "FAILURE"


def backTracking(graph, start_node_id, terminal_node_id):

    # Inicializando flag de sucesso e fracasso
    success = False
    fail = False
    # A solucao sera descoberta manipulando uma "pilha"
    solution = [start_node_id]
    visited = [start_node_id]

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
                if chsn_node == terminal_node_id:
                    success = True
                    break
        else:
            # Se voltarmos para o no inicial, significa que nao ha mais nos para serem explorados
            if current_node == start_node_id:
                fail = True
                break
            else:
                # Senao, voltamos na "arvore" de busca
                solution.pop()

    return solution, "SUCCESS" if success else "FAILURE"


def breadth_first_search(graph, start, destination):
    parentMap = {}
    visited = []
    solution = []
    current = start
    queue = deque()
    queue.append(current)
    visited.append(current)
    success = False

    while queue and not success:
        current = queue.popleft()
        if current == destination:
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
        while curr_id != start.vertex_id:
            solution.append(curr_id)
            curr_id = parentMap[curr_id]
        solution.append(curr_id)
        solution.reverse()
        return solution, "success"
    else:
        return solution, "failure"


def depth_first_search(graph, start, destination):
    parentMap = {}
    visited = []
    stack = []
    solution = []
    current = start
    stack.append(current)
    success = False

    while stack:
        current = stack[-1]
        stack.pop()

        if current not in visited:
            visited.append(current)

        if current == destination:
            success = True
            break

        for child in graph[current]:
            if child not in visited:
                stack.append(child)
                parentMap[child.vertex_id] = current.vertex_id

    curr_id = current.vertex_id
    if success:
        while curr_id != start.vertex_id:
            solution.append(curr_id)
            curr_id = parentMap[curr_id]
        solution.append(curr_id)
        solution.reverse()
        return solution, "success"
    else:
        return solution, "failure"


if __name__ == "__main__":
    G = Graph(False)
    nodes_list = list(string.ascii_uppercase)
    test_map = map_Generator(nodes_list, 1.1)
    print(test_map)
    for conection in test_map:
        G.addEdge(conection[0], conection[1], conection[2])
    print(G)
    print(backTracking(G, "B", "Z"))
    print(irrevocabile(G, "B", "Z"))