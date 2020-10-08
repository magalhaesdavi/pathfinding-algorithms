from .graph import Graph
from queue import Queue
import random


def irrevocabile(graph, start_node_id, terminal_node_id):
    pass


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

    i = 0

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
