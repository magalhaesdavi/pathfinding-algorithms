from GraphPackage import graph_script

g = graph_script.Graph() 
g.addEdge(1, 0, 5) 
g.addEdge(0, 2, 5) 
g.addEdge(2, 1, 5) 
g.addEdge(0, 3, 5) 
g.addEdge(1, 4, 5)  
  
# print("Following is Depth First Traversal")  
g.DFS(0)

# g = graph_script.Graph() 
# g.addEdge(0, 1) 
# g.addEdge(0, 2) 
# g.addEdge(1, 2) 
# g.addEdge(2, 0) 
# g.addEdge(2, 3) 
# g.addEdge(3, 3) 
  
# print ("Following is Breadth First Traversal"
#                   " (starting from vertex 2)") 
# g.BFS(2)
