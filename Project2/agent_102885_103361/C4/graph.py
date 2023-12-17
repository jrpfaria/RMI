import heapq
from collections import deque
from operator import itemgetter

class Graph:
    def __init__(self):
        self.nodes = set()
        self.visited = set()
        self.edges = {}
        self.beacons = {}
        self.beacon_count = 0
        self.start = None
        self.limits = None

    def set_limits(self, limits):
        self.limits = limits

    def set_start(self, start):
        self.start = start

    def set_beacon_count(self, count):
        self.beacon_count = count

    def unknown_nodes(self):
        return self.nodes.difference(self.visited)

    def beacon_edges(self):
        beacons = sorted(self.beacons.items(), key=itemgetter(0))

        beacon_edges = [] 
        for i in range(len(beacons) - 1):
            beacon_edges.append((beacons[i][1], beacons[i + 1][1]))

        return beacon_edges

    def _on_limits(self, node):
        limit_x, limit_y = self.limits
        node_x, node_y = node
        return (0 <= abs(node_x) <= limit_x) and (0 <= abs(node_y) <= limit_y)

    def add_node(self, node):
        if self._on_limits(node):
            self.nodes.add(node)
            if node not in self.edges:
                self.edges[node] = {}

    def add_edges_from(self, edges):
        for edge in edges:
            if len(edge) == 3:
                node1, node2, cost = edge
                self.add_edge(node1, node2, cost)
            else:
                node1, node2 = edge
                self.add_edge(node1, node2)

    def remove_node(self, node):
        self.nodes.discard(node)
        self.visited.discard(node)
        self.edges.pop(node, None)
        for n in self.edges:
            self.edges[n].pop(node, None)
    
    def remove_edge(self, node1, node2):
        self.edges.get(node1, {}).pop(node2, None)
        self.edges.get(node2, {}).pop(node1, None)

    def add_connections(self, node, nodes):
        for n in nodes:
            self.add_edge(node, n)

    def add_nodes_from(self, nodes):
        for node in nodes:
            self.add_node(node)

    def add_edge(self, node1, node2, cost=1):
        if self._on_limits(node1) and self._on_limits(node2):
            self.add_node(node1)
            self.add_node(node2)
            self.edges[node1][node2] = cost
            self.edges[node2][node1] = cost

    def add_beacon(self, number, node):
        if self._on_limits(node):
            self.beacons[number] = node
    
    def set_visited(self, node):
        if self._on_limits(node):
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

            for neighbor, cost in self.edges.get(current, {}).items():
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


    def astar_beacons(self):
        start = self.start
        beacon_edges = self.beacon_edges()

        print("beacon_edges: ", beacon_edges)

        last_node = start
        
        total_path = []
        for edge in beacon_edges:
            path, _ = self.astar(edge[0], edge[1])
            print("beacon path from ", edge[0], " to ", edge[1], ": ", path)
            if path:
                last_node = path[-1]
                total_path += path[:-1]

        if not total_path:
            return total_path, None
            
        path, goal = self.astar(last_node, start)

        if path:
            total_path += path
        
        return total_path, goal
            


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

            for neighbor, _ in self.edges.get(current, {}).items():
                if neighbor not in closed_set and neighbor not in open_queue:
                    came_from[neighbor] = current
                    open_queue.append(neighbor)

        return None, None

    def heuristic(self, node, goal, method='euclidean'):
        if method == 'euclidean':
            return ((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2) ** 0.5
        if method == 'manhattan':
            return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.insert(0, current)
        return path