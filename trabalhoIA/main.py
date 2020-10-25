from collections import namedtuple
import searchAlgorithms as sal
from graph import Graph

g = Graph()

vertex = namedtuple("Vertex", ["vertex_id", "vertex_x", "vertex_y"])
a = vertex(vertex_id = 'A', vertex_x = 2, vertex_y = 3)
b = vertex(vertex_id = 'B', vertex_x = 5, vertex_y = 4)
c = vertex(vertex_id = 'C', vertex_x = 7, vertex_y = 8)
d = vertex(vertex_id = 'D', vertex_x = 15, vertex_y = 9)
e = vertex(vertex_id = 'E', vertex_x = 15, vertex_y = 9)

g.add_edge(a, b, 5)
g.add_edge(a, c, 9)
g.add_edge(c, d, 9)
g.add_edge(b, e, 9)
# g.addEdge(c, e, 9)
# g.addEdge(w, u, 9)
# g.addEdge(w, r, 9)
# g.addEdge(r, r, 9)
print(g)
print(sal.breadth_first_search(g, vertex(vertex_id = 'A', vertex_x = 2, vertex_y = 3), e))
