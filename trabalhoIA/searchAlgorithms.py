from collections import defaultdict, deque, namedtuple, OrderedDict
from queue import Queue, PriorityQueue
from faker import Faker
import math
import random
import string
import timeit
from graph import Graph
import utils

def irrevocable(graph, start_id, end_id):

    start_node = [node for node in list(graph.graph.keys()) if node.vertex_id == start_id][0]

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
            chosen_node = edges[0]

            if chosen_node not in visited:
                visited.append(chosen_node)
                if chosen_node not in solution:
                    ok = True
                    break
            else:
                del edges[0]

        if not ok:
            fail = True
            break

        solution.append(chosen_node)
        if chosen_node.vertex_id == end_id:
            success = True
            break

    return [node.vertex_id for node in solution], "SUCCESS" if success else "FAILURE"

def backTracking(graph, start_id, end_id):

    start_node = [node for node in list(graph.graph.keys()) if node.vertex_id == start_id][0]

    # Variaveis de estatisticas
    start_time = timeit.default_timer()
    cost = 0
    expanded = 0
    branching_factor = []

    # Inicializando flag de sucesso e fracasso
    success = False
    fail = False
    # A solucao sera descoberta manipulando uma "pilha"
    solution = [start_node]
    visited = [start_node]

    while not success or not fail:
        # Verificamos se e possivel aplicar regras no ultimo no na pilha
        current_node = solution[-1]
        edges = list(graph[current_node].keys())

        # Se ainda existe pelo menos uma aresta que ainda não foi visitada entramos no if
        if not all(node in visited for node in edges):

            # Nesta implementacao a escolha da regra/edge é por ordem alfabética ou crescente (se forem numeros)
            edges.sort()
            chosen_node = edges[0]
            while chosen_node in visited:
                chosen_node = edges[edges.index(chosen_node) + 1]

            branching_factor.append(1)
            expanded += 1
            if chosen_node not in visited:
                visited.append(chosen_node)

            if chosen_node not in solution:
                solution.append(chosen_node)
                cost += graph[current_node][chosen_node].weight
                # Se o proximo no for o no terminal, ativamos a flag de sucesso e terminamos o loop
                if chosen_node.vertex_id == end_id:
                    success = True
                    break
        else:
            # Se voltarmos para o no inicial, significa que nao ha mais nos para serem explorados
            if current_node.vertex_id == start_id:
                fail = True
                break
            else:
                # Senao, voltamos na "arvore" de busca
                poped_node = solution.pop()
                cost -= graph[poped_node][solution[-1]].weight
                del poped_node


    end_time = timeit.default_timer()
    depth = len(solution) - 1
    exec_time = end_time - start_time
    average_branching_factor = sum(branching_factor) / len(branching_factor)


    return [node.vertex_id for node in solution], depth, cost, expanded, len(visited), average_branching_factor, exec_time, "success" if success else "fail"

def breadth_first_search(graph, start_id, end_id):

    start_time = timeit.default_timer()
    expanded = 0
    branching_factor = []

    start_node = [node for node in list(graph.graph.keys()) if node.vertex_id == start_id][0]
    end_node = [node for node in list(graph.graph.keys()) if node.vertex_id == end_id][0]
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
            expanded += 1
            n = 0
            for child in graph[current]:
                if child not in visited:
                    queue.append(child)
                    visited.append(child)
                    parentMap[child] = current
                n += 1
            branching_factor.append(n)

    cost = 0
    if success:
        while current != start_node:
            solution.append(current.vertex_id)
            cost += graph[current][parentMap[current]].weight
            current = parentMap[current]
        solution.append(current.vertex_id)
        solution.reverse()

        end_time = timeit.default_timer()
        depth = len(solution) - 1
        exec_time = end_time - start_time
        average = sum(branching_factor) / len(branching_factor)

        return solution, depth, cost, expanded, len(visited), average, exec_time, 'success'
    else:
        end_time = timeit.default_timer()
        exec_time = end_time - start_time
        return solution, -1, cost, expanded, len(visited), -1, exec_time, 'failure'

def depth_first_search(graph, start_id, end_id):

    start_time = timeit.default_timer()
    expanded = 0
    branching_factor = []

    start_node = [node for node in list(graph.graph.keys()) if node.vertex_id == start_id][0]
    end_node = [node for node in list(graph.graph.keys()) if node.vertex_id == end_id][0]

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

        else:
            expanded += 1
            n = 0
            for child in graph[current]:
                if child not in visited:
                    stack.append(child)
                    parentMap[child] = current
                n +=1
            branching_factor.append(n)
    # curr_id = current.vertex_id
    cost = 0
    if success:
        while current != start_node:
            solution.append(current.vertex_id)
            cost += graph[current][parentMap[current]].weight
            current = parentMap[current]
        solution.append(current.vertex_id)
        solution.reverse()

        end_time = timeit.default_timer()
        depth = len(solution) - 1
        exec_time = end_time - start_time
        average = sum(branching_factor) / len(branching_factor)
        return solution, depth, cost, expanded, len(visited), average, exec_time, 'success'
    else:
        end_time = timeit.default_timer()
        exec_time = end_time - start_time
        return solution, -1, cost, expanded, len(visited), -1, exec_time, 'failure'

def uniform_cost_search(graph, start_id, end_id):

    start_node = [node for node in list(graph.graph.keys()) if node.vertex_id == start_id][0]
    end_node = [node for node in list(graph.graph.keys()) if node.vertex_id == end_id][0]

    # Variaveis de estatisticas
    start_time = timeit.default_timer()
    expanded = 0
    branching_factor = []

    success = False
    fail = False

    solution = []
    visited = []

    min_cost = float("inf")
    cost = 0
    
    prority_queue = PriorityQueue()
    path_to_goal = []
    prority_queue.put((0, start_node, path_to_goal))


    while not prority_queue.empty():

        cost, current_node, current_path = prority_queue.get()

        if current_node.vertex_id == end_id and cost < min_cost:
            success = True
            solution = current_path + [end_node]
            min_cost = cost
        else:
            if not current_node in visited:

                edges = list(graph[current_node].keys())

                branching_factor.append(len(edges))

                while len(edges) > 0:
                    edge = edges.pop()
                    prority_queue.put(
                        (cost + graph[current_node][edge].weight, edge, current_path + [current_node])
                    )

                visited.append(current_node)
            expanded += 1
    
    end_time = timeit.default_timer()
    depth = len(solution) - 1
    exec_time = end_time - start_time
    average_branching_factor = sum(branching_factor) / len(branching_factor)

    return [node.vertex_id for node in solution], depth, min_cost, expanded, len(visited), average_branching_factor, exec_time, "success" if success else "fail"

def greedy(graph, start_id, end_id):

    start_time = timeit.default_timer()
    expanded = 0
    branching_factor = []

    start_node = [node for node in list(graph.graph.keys()) if node.vertex_id == start_id][0]
    end_node = [node for node in list(graph.graph.keys()) if node.vertex_id == end_id][0]

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
        else:
            expanded +=1
            n = 0
            children = {}
            for child in graph[current]:
                if child not in visited:
                    children[child] = utils.heuristic(child, end_node)
                    parentMap[child] = current
                n += 1
            branching_factor.append(n)

            if children:
                sorted_children = list({k: v for k, v in sorted(children.items(), key=lambda item: item[1])}.keys())
                sorted_children.reverse()
                stack += sorted_children

    cost = 0
    if success:
        while current != start_node:
            solution.append(current.vertex_id)
            cost += graph[current][parentMap[current]].weight
            current = parentMap[current]
        solution.append(current.vertex_id)
        solution.reverse()

        end_time = timeit.default_timer()
        depth = len(solution) - 1
        exec_time = end_time - start_time
        average = sum(branching_factor) / len(branching_factor)

        return solution, depth, cost, expanded, len(visited), average, exec_time, 'success'
    else:
        end_time = timeit.default_timer()
        exec_time = end_time - start_time
        return solution, -1, cost, expanded, len(visited), -1, exec_time, 'failure'

def a_star(graph, start_id, end_id):

    start_time = timeit.default_timer()
    expanded = 0
    branching_factor = []

    start_node = [node for node in list(graph.graph.keys()) if node.vertex_id == start_id][0]
    end_node = [node for node in list(graph.graph.keys()) if node.vertex_id == end_id][0]

    openList = {}
    closedList = {}

    # A lista contém respectivamente: g(custo acumulado), funcao h da heuristica e funcao f(g + h)
    openList[start_node] = [0, utils.heuristic(
        start_node, end_node), 0 + utils.heuristic(start_node, end_node)]

    success = False
    solution = []
    parentMap = {}

    while openList:
        current = utils.find_smaller(openList, 'a_star')
        closedList[current] = openList[current]
        del openList[current]
        if current == end_node:
            success = True
            break
        else:
            expanded += 1
            n = 0
            for child in graph[current]:
                if child in closedList:
                    continue
                if child in openList:
                    new_g = closedList[current][0] + graph[current][child].weight
                    if openList[child][0] > new_g:
                        openList[child][0] = new_g
                        openList[child][2] = new_g + openList[child][1]
                        parentMap[child] = current
                else:
                    child_g = closedList[current][0] + graph[current][child].weight
                    child_h = utils.heuristic(child, end_node)
                    openList[child] = [child_g, child_h, child_g + child_h]
                    parentMap[child] = current
                n += 1
            branching_factor.append(n)

    cost = 0
    if success:
        while current != start_node:
            solution.append(current.vertex_id)
            cost += graph[current][parentMap[current]].weight
            current = parentMap[current]
        solution.append(current.vertex_id)
        solution.reverse()

        end_time = timeit.default_timer()
        depth = len(solution) - 1
        exec_time = end_time - start_time
        average = sum(branching_factor) / len(branching_factor)

        return solution, depth, cost, expanded, len(openList) + len(closedList), average, exec_time, 'success'
    else:
        end_time = timeit.default_timer()
        exec_time = end_time - start_time
        return solution, -1, cost, expanded, len(visited), -1, exec_time, 'failure'

def ida_star(graph, start_id, end_id):

    start_node = [node for node in list(graph.graph.keys()) if node.vertex_id == start_id][0]
    end_node = [node for node in list(graph.graph.keys()) if node.vertex_id == end_id][0]

    #Variaveis de estatisticas
    start_time = timeit.default_timer()
    expanded = [0]
    cost = 0
    visited = []
    branching_factor = []

    success = False
    fail = False
    solution = []

    limit = utils.heuristic(start_node, end_node)

    while not success or not fail:
        distance = ida_star_aux(
                        graph,
                        start_node,
                        end_node,
                        0,
                        visited,
                        limit,
                        solution,
                        expanded,
                        branching_factor
                    )

        if distance == float("inf"):
            fail = True
            break
        elif distance < 0:
            success = True
            break
        else:
            limit = distance
            solution = []
            visited = []
            expanded[0] = 0
            branching_factor.clear()

    end_time = timeit.default_timer()
    cost = -1 * distance
    depth = len(solution) - 1
    exec_time = end_time - start_time
    average_branching_factor = sum(branching_factor) / len(branching_factor)

    return [node.vertex_id for node in solution], depth, cost, expanded[0], len(visited), average_branching_factor, exec_time, "success" if success else "fail"

def ida_star_aux(graph, node, end_node, distance, visited, limit, path, expanded, branching_factor):

    path.append(node)
    visited.append(node)

    if node == end_node:
        return -distance

    estimate = distance + utils.heuristic(node, end_node)
    if estimate > limit:
        path.pop()
        return estimate

    n_limit = float("inf")
    edges = list(graph[node].keys())
    edges.sort(key=lambda edge: edge.vertex_id)

    expanded[0] += 1

    for edge in edges:
        if edge not in visited:

            edge_dist = ida_star_aux(
                            graph,
                            edge,
                            end_node,
                            distance + graph[node][edge].weight,
                            visited,
                            limit,
                            path,
                            expanded,
                            branching_factor
                        )

            if edge_dist < 0:
                branching_factor.append(1)
                return edge_dist
            elif edge_dist < n_limit:
                n_limit = edge_dist

    branching_factor.append(1)

    if len(path) > 0:
        path.pop()
    return n_limit