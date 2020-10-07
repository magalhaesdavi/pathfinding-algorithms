from collections import defaultdict
from collections import deque

#O grafo é um defaultdict de dict, no dict interior temos o vertice conectado com o valor do peso da aresta

class Graph:

    def __init__(self):
        # self.V = 0
        # self.node = defaultdict(int)
        self.graph = defaultdict(dict)

    def addEdge(self, u, v, weight):
        # if (u not in self.graph) and (v not in self.graph):
        #     self.V += 2
        #     print("dois")
        # else:
        #     if (u in self.graph) or (v in self.graph):
        #         self.V += 1


        # self.graph[u].append(v)
        if u not in self.graph: #esse if é para nao resetar o no toda vez q for inserir uma aresta nele
            self.graph[u] = {}          
        self.graph[u][v] = weight

        if v not in self.graph:
            self.graph[v] = {}

    def BFS(self, s):
        visited = [False] * (len(self.graph))
        queue = deque()
        queue.append(s)
        visited[s] = True

        while queue:
            s = queue.popleft()
            print(s, end = " ")

            for i in self.graph[s]:
                if visited[i] == False:  #not visited[i] talvez funcione também
                    queue.append(i)
                    visited[i] = True
    
    def DFS(self, s):
        visited = [False] * (len(self.graph))
        stack = []
        stack.append(s)

        while stack:
            s = stack[-1]
            stack.pop()

            if not visited[s]:  #not visited[s] talvez funcione também
                print(s, end = " ")
                visited[s] = True

            for i in self.graph[s]:
                if not visited[i]:  #not visited[i] talvez funcione também
                    stack.append(i)
