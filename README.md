# Pathfinding algorithms implementation

In this work we analyze the performance of the main search algorithms in the literature on the map problem.

The problem consists of finding the shortest path (with the lowest cost) between two points on a map.

A graph class for the representation of maps was implemented, as well as a random instance generator to carry out the experiments.

Instance example:

<img src="./notebooks/graph_n25.png"/>

## Algorithms

Implemented algorithms:

- Irrevogável;

- Backtracking;

- Breadth-first search;

- Depth-first search;

- Uniform-Cost Search;

- A\*;

- IDA\*

## Execution

To run the program, just access the terminal in the project folder and run the command:

`python main.py`

and the program will run all the search algorithms 10 times for each instance, with instances having sizes of 25, 50, 100 and 200 nodes. The results will be stored as "results.csv" in the "outputs" folder.
