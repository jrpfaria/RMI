import heapq
from collections import deque

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
                return path, current

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

        return None, None


    def bfs_unknowns(self, start):
        goals = self.unknown_nodes()  # Set of unknown nodes

        if not goals:
            return None, None
        
        if start in goals:
            return [start], start

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

        return None, None

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