import heapq
from collections import deque

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

    def add_nodes_from(self, nodes):
        self.nodes.update(nodes)

    def add_edge(self, node1, node2, cost):
        self.edges.setdefault(node1, []).append((node2, cost))
        self.edges.setdefault(node2, []).append((node1, cost))

    def add_beacon(self, number, node):
        self.beacons.add((number, node))

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

    def bfs_unknown(self, start):
        goals = self.unknown_nodes()  # Set of unknown nodes

        open_queue = deque([start])  # Queue for nodes to be evaluated
        closed_set = set()  # Set of nodes already evaluated
        came_from = {}  # Mapping of nodes to their predecessors

        while open_queue:
            current = open_queue.popleft()

            if current in goals:
                path = self.reconstruct_path(came_from, current)
                return path, current  # Return the path and the closest goal

            closed_set.add(current)

            for neighbor, _ in self.edges.get(current, []):
                if neighbor not in closed_set and neighbor not in open_queue:
                    came_from[neighbor] = current
                    open_queue.append(neighbor)

        return None

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