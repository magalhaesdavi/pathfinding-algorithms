from collections import defaultdict
from collections import deque

class Graph:

    def __init__(self):
        # self.V = 0
        self.graph = defaultdict(list)

    def addEdge(self, u, v):
        # if (u not in self.graph) and (v not in self.graph):
        #     self.V += 2
        #     print("dois")
        # else:
        #     if (u in self.graph) or (v in self.graph):
        #         self.V += 1
        self.graph[u].append(v)
        # print(self.graph[v])
        if v not in self.graph:
            self.graph[v] = []
        # print(self.graph)
        # print(self.V)      

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
        # print(self.graph)
        # print(len(self.graph))
        visited = [False] * (len(self.graph))
        # print(visited)
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
