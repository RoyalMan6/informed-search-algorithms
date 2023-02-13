
import json
import haversine as hs

with open(r"/Users/mohammedalmousa/Downloads/AllReg 2.json", "r", encoding="utf8") as f:
    graphjson = json.loads(f.read())
    graph = {}
    graph1 = {}
    vertices_no = 0
    cities = []

def add_vertex(v):
    global graph
    global vertices_no
    vertices_no = vertices_no + 1
    graph[v["name"]] = []
    graph1[v["name"]] = []
# -----------------------------------------------------------------
def add_edge(v1, v2, e):  # Add an edge between vertex v1 and v2 with edge weight e
    global graph

    # Since this code is not restricted to a directed or an undirected graph, an edge between v1 v2 does not imply that
    # an edge exists between v2 and v1
    temp = [v2, e]
    temp1 = [v2]
    graph[v1["name"]].append(temp)
    graph1[v1["name"]].extend(temp1)
# -----------------------------------------------------------------
for u in graphjson:
    add_vertex(u)

for u1 in graphjson:
    v1 = u1
    for tr in v1["neighbors"]:
        v2 = graphjson[tr["cid"]]["name"]
        e = tr["distance"]
        add_edge(v1, v2, e)

path = []
for i in range (0, len(graphjson)):
    cities.append(graphjson[i]["name"])

def get_cid(city):
    x = len(graphjson)
    for a in range(x):
        if(graphjson[a]["name"] == city):
            return a
    return -1
#------------------------------------------------------------------------------------------------------------------#
def H_value(node, goal):
    loc1 = (graphjson[get_cid(node)]["X"], graphjson[get_cid(node)]["Y"])
    loc2 = (graphjson[get_cid(goal)]["X"], graphjson[get_cid(goal)]["Y"])
    dist = hs.haversine(loc1,loc2)
    return dist

aCity = NULL
gPathCost = NULL
aPathCost = NULL

def path_cost(path):
        total_cost = 0
        for (node, cost) in path:
            total_cost += cost
        return round(total_cost, 2)


def path_h_cost(path):
    g_cost = 0
    for (node, cost) in path:
        g_cost += cost
    global gPathCost
    gPathCost = g_cost
    last_node = path[-1][0]
    h_cost = H_value(last_node, aCity)
    return h_cost, last_node

def Greedy(graph, start, goal):
    global aCity
    aCity = goal
    visited = []
    queue = [[(start, 0)]]
    Greedy.num_node = 1
    Greedy.max_fringe = 1
    if start == goal:
        return 'No solution'
    while queue:
        queue.sort(key=path_h_cost)  # sort by cost
        path = queue.pop(0)
        node = path[-1][0]
        if node not in visited:
            adj_nodes = graph[node]
            Greedy.num_node += len(adj_nodes)
            if len(queue) + len(adj_nodes) > Greedy.max_fringe:
                Greedy.max_fringe = len(queue) + len(adj_nodes)
            visited.append(node)
            if node == goal:
                return path
            else:
                for (node2, cost) in adj_nodes:
                    new_path = path.copy()
                    new_path.append((node2, cost))
                    queue.append(new_path)

def path_f_cost(path):
    g_cost = 0
    for (node, cost) in path:
        g_cost += cost
    global aPathCost
    aPathCost = g_cost
    last_node = path[-1][0]
    h_cost= H_value(last_node, aCity)
    f_cost = g_cost + h_cost
    return f_cost, last_node

def A_star(graph, start, goal):
    visited = []
    queue = [[(start, 0)]]
    A_star.num_node = 1
    A_star.max_fringe = 1
    if start == goal:
        return 'No solution'
    while queue:
        queue.sort(key=path_f_cost)  # sort by cost
        path = queue.pop(0)
        node = path[-1][0]
        if node not in visited:
            adj_nodes = graph[node]
            A_star.num_node += len(adj_nodes)
            if len(queue) + len(adj_nodes) > A_star.max_fringe:
                A_star.max_fringe = len(queue) + len(adj_nodes)
            visited.append(node)
            if node == goal:
                return path
            else:
                for (node2, cost) in adj_nodes:
                    new_path = path.copy()
                    new_path.append((node2, cost))
                    queue.append(new_path)

def cost(distance, economy):
    return round(distance / economy * 2.18, 2)
#---------------------------------- GUI ----------------------------------#

dCity = input('Enter your start city:')
aCity = input('Enter your goal city:')
economy = input('fuel E')
gRoute = Greedy(graph, dCity, aCity)
gDistance = path_cost(Greedy(graph, dCity, aCity))
gCost = cost(gDistance, economy)

aRoute = A_star(graph, dCity, aCity)
aDistance = path_cost(A_star(graph, dCity, aCity))
aCost = cost(aDistance, economy)

print("Greedy Search:\n Route: " + str(gRoute) + "\n Distance: " + str(gDistance) + " KM\n Cost: " + str(gCost) + " SAR" +
            "\n Generated nodes: " + str(Greedy.num_node) + "\n Maximum fringe size: " + str(Greedy.max_fringe) +
            "\n\n A* Search:\n Route: " + str(aRoute) + "\n Distance: " + str(aDistance) + " KM\n Cost: " + str(aCost) + " SAR"
            "\n Generated nodes: " + str(A_star.num_node) + "\n Maximum fringe size: " + str(A_star.max_fringe))
