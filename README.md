# Informed-search-algorithms

Algorithms were used to find the fastest route between the two cities in the Kingdom. Where the algorithms take the map as an input

Greedy search and A* are both informed search algorithms, also known as heuristic search algorithms. They both use information about the search space to guide the search towards the goal.

The Greedy search algorithm selects the next node to explore based on the closest estimated distance to the goal. It does not consider the cost of reaching the node, only the estimated distance from the node to the goal.

A* (A-star) is a more sophisticated informed search algorithm that combines the strengths of both Greedy search and Dijkstra's algorithm. A* uses a cost function that takes into account both the cost of reaching the node and the estimated distance from the node to the goal. A* is guaranteed to find the optimal solution if the heuristic function used is admissible (never overestimates the distance to the goal) and consistent (satisfies the triangle inequality).

Both Greedy search and A* are commonly used in pathfinding and navigation problems in AI and computer science. The choice of which one to use depends on the specific requirements of the problem being solved.
