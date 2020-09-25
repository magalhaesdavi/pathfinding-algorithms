from collections import defaultdict
from collections import deque

class Graph:

    def __init__(self):
        self.graph = defaultdict(list)

    def addEdge(self, u, v):
        self.graph[u].append(v)

    def BFS(self, s):
        visited = False * (len(self.graph))
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
        visited = False * (len(self.graph))
        stack = []
        stack.append(s)
        print(type(visited))
        visited[s] = True

        while stack:
            s = stack[-1]
            stack.pop()

            if visited[s] == False:  #not visited[s] talvez funcione também
                print(s, end = " ")
                visited[s] = True

            for i in self.graph[s]:
                if visited[i] == False:  #not visited[i] talvez funcione também
                    stack.append(i)
