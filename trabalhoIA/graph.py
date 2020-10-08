from collections import defaultdict, deque, namedtuple
import random


class Graph:
    """
    Classe de grafo baseado em lista de adjacencia.
    O grafo e um defaultdict de dict,
    no dict interior temos o vertice conectado com o valor do peso da aresta.
    """

    def __init__(self, direcionado=True):
        # self.V = 0
        # self.node = defaultdict(int)
        self.__direcionado = direcionado
        self.graph = defaultdict(dict)

    def __len__(self):
        return len(self.graph)

    def __getitem__(self, key):
        if key not in self.graph:
            raise KeyError("Key {} not in the graph.".format(key))
        return self.graph[key]

    def __iter__(self):
        return self.graph.items()

    def __contains__(self, value):
        return value in self.graph

    def __str__(self):
        return self.plotGraph()

    def searchNode(self, node_id):
        """
        Procura e retorna o no pelo id 'node_id e retorna False caso não encontre.
        """
        return self.graph[node_id] if node_id in self.graph else False

    def insertNode(self, node_id):
        """
        Insere um nó de id 'node_id' no grafo.
        A operação é cancelada caso o nó já exista.
        """
        if self.searchNode(node_id):
            return "ERROR! Node already in the graph"
        else:
            self.graph[node_id] = {}
            return True

    def addEdge(self, u, v, weight_=0, edgeID_=-1, inplace=False):
        """
        Conecta dois nos de id 'u' e 'v' atraves de uma arestas com um peso 'weight' associado.
        Caso os nos nao existam, os mesmos sao criados.
        O paramêtro 'inplace' especifica se, caso um dos nós não exista, os mesmos sejam criados.
        """

        # if (u not in self.graph) and (v not in self.graph):
        #     self.V += 2
        #     print("dois")
        # else:
        #     if (u in self.graph) or (v in self.graph):
        #         self.V += 1

        # self.graph[u].append(v)
        # esse if é para nao resetar o no toda vez q for inserir uma aresta nele
        if u not in self.graph:
            if inplace:
                self.graph[u] = {}
            else:
                return f"ERROR! Node {u} not in the graph"

        # ESSE É O JEITO CERTO, NÃO TENTE ME CONFRONTAR
        edge = namedtuple("Edge", ["edgeID", "weight"])
        self.graph[u][v] = edge(edgeID=edgeID_, weight=weight_)
        # self.graph[u][v] = weight

        if not self.__direcionado:
            if v not in self.graph:
                if inplace:
                    self.graph[v] = {}
                else:
                    return f"ERROR! Node {v} not in the graph"

            self.graph[v][u] = edge(edgeID=edgeID_, weight=weight_)
            # self.graph[v][u] = weight

        return True

    def plotGraph(self):
        plot = "######################\n"
        for node in self.graph.items():
            plot += f"{node[0]} -> "
            for edge in node[1].items():
                plot += f"({edge[0]}, {edge[1].weight}), "
            plot += "\n"
        plot += "######################\n"
        return plot

    def BFS(self, s):
        visited = [False] * (len(self.graph))
        queue = deque()
        queue.append(s)
        visited[s] = True

        while queue:
            s = queue.popleft()
            print(s, end=" ")

            for i in self.graph[s]:
                if visited[i] == False:  # not visited[i] talvez funcione também
                    queue.append(i)
                    visited[i] = True

    def DFS(self, s):
        visited = [False] * (len(self.graph))
        stack = []
        stack.append(s)

        while stack:
            s = stack[-1]
            stack.pop()

            if not visited[s]:  # not visited[s] talvez funcione também
                print(s, end=" ")
                visited[s] = True

            for i in self.graph[s]:
                if not visited[i]:  # not visited[i] talvez funcione também
                    stack.append(i)


if __name__ == "__main__":
    G = Graph(False)
    G.insertNode("A")
    G.insertNode("B")
    G.addEdge("A", "B", 10)

    print(len(G))
    print(G["B"])
    print(G.searchNode("A"))
    print(G.graph)
    print(G)