# from graph import Graph
from queue import Queue
from collections import defaultdict, deque, namedtuple
import random
import time 

def backTracking(graph, start_node_id, terminal_node_id):
    # Inicializando flag de sucesso e fracasso
    success = False
    fail = False
    # A solucao sera descoberta manipulando uma "pilha"
    solution = []
    tree = []
    # Primeiro selecionamos o no de partida
    # startNode = graph.buscaNo(start_node_id)
    # E colocamos ele na pilha de solucao
    solution.append(start_node_id)
    tree.append(start_node_id)

    # i = 0

    while not success or not fail:
        # Verificamos se e possivel aplicar regras no ultimo no na pilha
        currentNode = tree[-1]

        # if len(currentNode.arestas) > 0:
        if len(graph[start_node_id]) > 0:
            randN = random.randint(0, len(graph[start_node_id]) - 1)
            # Nesta implementacao a escolha da regra sera aleatoria
            # if currentNode.arestas[randN].getNoAdj() not in solution:
            if graph[currentNode][randN] not in solution:
                # nextNode = graph.buscaNo(currentNode.arestas[randN].getNoAdj())
                nextNode = graph[currentNode][randN]
                tree.append(nextNode)
                solution.append(nextNode)
                # Se o proximo no for o no terminal, ativamos a flag de sucesso e terminamos o loop
                if nextNode == terminal_node_id:
                    success = True
                    break

            else:
                # Se voltarmos para o no inicial, significa que nao ha mais nos para serem explorados
                if currentNode == start_node_id:
                    fail = True
                    break
                else:
                    # Senao, voltamos na "arvore" de busca
                    tree.pop()
                    solution.pop()

    return solution, "SUCCESS" if success else "FAILURE"

def breadth_first_search(graph, start, end):

    # start = time.time()
    start_t = time.time()
    path = list({k for k in graph.graph.keys() if k.vertex_id == start})
    path.append(list({k for k in graph.graph.keys() if k.vertex_id == end})[0])
    end_t = time.time()

    print("")
    print("TIME: ", end_t - start_t)
    print(path)
    # print(end_t - start_tart)
    # print(path)

    # parentMap = {}
    # visited = []
    # solution = []
    # # current = start
    # queue = deque()
    # queue.append(current)
    # visited.append(current)
    # success = False
    # # print(current)
    # # print(destination)
        
    # while queue and not success:
    #     current = queue.popleft()   
    #     if current == destination:
    #         success = True
    #         break
    #     else:
    #         print(current)
    #         for child in graph[current]:
    #             if child not in visited:
    #                 queue.append(child)
    #                 visited.append(child)
    #                 parentMap[child.vertex_id] = current.vertex_id
        
    # curr_id = current.vertex_id
    # if success:
    #     while curr_id != start.vertex_id:
    #         solution.append(curr_id)
    #         curr_id = parentMap[curr_id]
    #     solution.append(curr_id)
    #     solution.reverse()
    #     return solution, "success"
    # else:
    #     return solution, "failure"

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


def a_star(graph, start, destination):
    pass