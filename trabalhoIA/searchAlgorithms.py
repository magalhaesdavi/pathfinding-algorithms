from graph import Graph
from utils import map_Generator
from queue import Queue
import random
import string


def irrevocabile(graph, start_node_id, terminal_node_id):
    # Inicializando flag de sucesso e fracasso
    success = False
    fail = False
    solution = [start_node_id]
    ok = False

    while not success or not fail:
        currentNode = solution[-1]
        edges = [node for node in graph[currentNode].keys()]
        while len(edges) > 0:
            # Nesta implementacao a escolha da regra sera aleatoria
            randNode = random.sample(edges, 1)[0]
            next_state = randNode
            if randNode not in solution:
                ok = True
                break
            else:
                edges.pop(edges.index(randNode))

        if not ok:
            fail = True
            break
        solution.append(next_state)
        if next_state == terminal_node_id:
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
        currentNode = solution[-1]
        edges = list(G[currentNode].keys())
        # EU PRECISO DE UM IF AQUI
        if not all(node in visited for node in edges):
            # Nesta implementacao a escolha da regra sera aleatoria
            randNode = random.sample(edges, 1)[0]
            if randNode not in visited:
                visited.append(randNode)
            if randNode not in solution:
                solution.append(randNode)
                # Se o proximo no for o no terminal, ativamos a flag de sucesso e terminamos o loop
                if randNode == terminal_node_id:
                    success = True
                    break
        else:
            # Se voltarmos para o no inicial, significa que nao ha mais nos para serem explorados
            if currentNode == start_node_id:
                fail = True
                break
            else:
                # Senao, voltamos na "arvore" de busca
                solution.pop()

    return solution, "SUCCESS" if success else "FAILURE"


if __name__ == "__main__":
    G = Graph(False)
    nodes_list = list(string.ascii_uppercase)
    test_map = map_Generator(nodes_list, 1)
    print(test_map)
    for conection in test_map:
        G.addEdge(conection[0], conection[1], conection[2], inplace=True)
    print(G)
    print(backTracking(G, "B", "Z"))
    # print(irrevocabile(G, "B", "Z"))
