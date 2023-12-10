import heapq
from collections import deque
from itertools import permutations

class Node:
    def __init__(self, x, y):
        self.coordinates = (x, y)

    def __iter__(self):
        return iter(self.coordinates)

    def __cmp__(self, other):
        return isinstance(other, Node) and self.coordinates == other.coordinates

    def __eq__(self, other):
        return isinstance(other, Node) and self.coordinates == other.coordinates

    def __hash__(self):
        return hash(self.coordinates)

    def __lt__(self, other):
        return isinstance(other, Node) and self.coordinates < other.coordinates

    def __str__(self):
        return str(self.coordinates)

    def __repr__(self):
        return str(self)

    def __getitem__(self, item):
        return self.coordinates[item]
    
class Graph:
    def __init__(self):
        self.nodes = set()
        self.visited = set()
        self.edges = {}
        self.beacons = set()

    def unknown_nodes(self):
        return self.nodes.difference(self.visited)

    def add_node(self, node):
        self.nodes.add(node)
        if node not in self.edges:
            self.edges[node] = []

    def add_edges_from(self, edges):
        for edge in edges:
            if len(edge) == 3:
                node1, node2, cost = edge
                self.add_edge(node1, node2, cost)
            else:
                node1, node2 = edge
                self.add_edge(node1, node2)
    
    def add_connections(self, node, nodes):
        for n in nodes:
            self.add_edge(node, n)

    def add_nodes_from(self, nodes):
        for node in nodes:
            self.add_node(node)

    def add_edge(self, node1, node2, cost=1):
        self.add_node(node1)
        self.add_node(node2)
        self.edges[node1].append((node2, cost))
        self.edges[node2].append((node1, cost))

    def add_beacon(self, number, node):
        self.add_node(node)
        self.beacons.add((number, node))
    
    def set_visited(self, node):
        self.add_node(node)
        self.visited.add(node)

    def astar(self, start, goal):
        open_set = []   # Priority queue for nodes to be evaluated
        closed_set = set()  # Set of nodes already evaluated
        came_from = {}  # Mapping of nodes to their predecessors
        g_score = {node: float('inf') for node in self.nodes}  # Cost from start along best known path
        g_score[start] = 0
        f_score = {node: float('inf') for node in self.nodes}  # Estimated total cost from start to goal through y
        f_score[start] = self.heuristic(start, goal)

        heapq.heappush(open_set, (f_score[start], start))

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                path = self.reconstruct_path(came_from, current)
                return path

            closed_set.add(current)

            for neighbor, cost in self.edges.get(current, []):
                if neighbor in closed_set:
                    continue

                tentative_g_score = g_score[current] + cost

                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)

                    if neighbor not in open_set:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None

    def tsp(self, start):
        unknown_nodes = list(self.unknown_nodes())
        if not unknown_nodes:
            return 0, [start]

        shortest_path_length = float('inf')
        shortest_path = None

        for perm in permutations(unknown_nodes):
            path = [start] + list(perm)
            path_length = self.calculate_path_length(path)

            if path_length < shortest_path_length:
                shortest_path_length = path_length
                shortest_path = path

        return shortest_path_length, shortest_path

    def calculate_path_length(self, path):
        length = 0
        for i in range(len(path) - 1):
            node1, node2 = path[i], path[i + 1]
            edge = next((e for e in self.edges[node1] if e[0] == node2), None)
            if edge:
                length += edge[1]
        return length

        min_path, min_cost = min(all_paths, key=lambda x: x[1])
        return min_path, min_cost

    def heuristic(self, node, goal, method='euclidean'):
        if method == 'euclidean':
            return ((node.coordinates[0] - goal.coordinates[0]) ** 2 + (node.coordinates[1] - goal.coordinates[1]) ** 2) ** 0.5
        if method == 'manhattan':
            return abs(node.coordinates[0] - goal.coordinates[0]) + abs(node.coordinates[1] - goal.coordinates[1])

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.insert(0, current)
        return path