import heapq  # For priority queue

class Node:
    def __init__(self, x, y):
        self.coordinates = (x, y)
        self.g_score = float('inf')  # Initialize to infinity
        self.h_score = 0  # Heuristic score
        self.parent = None
        
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
    
class Graph:
    def __init__(self):
        self.nodes = set()
        self.edges = {}
        self.closed_nodes = set()
        self.open_nodes = set()
        self.beacon_count = 0
        self.beacon_nodes = set()

    def set_node_beacon_count(self, count):
        self.beacon_count = count

    def set_node_beacon(self, node, number):
        self.beacon_nodes.add((number, node))
        
    def set_node_visited(self, node):
        self.closed_nodes.add(node)
        self.open_nodes.remove(node)
        
    def set_node_unknown(self, node):
        self.open_nodes.add(node)
        self.closed_nodes.remove(node)

    def add_node(self, node):
        self.nodes.add(node)
        if node not in self.closed_nodes:
            self.open_nodes.add(node)

    def add_edge(self, node1, node2, distance = 2):
        self.edges[(node1, node2)] = distance
        self.edges[(node2, node1)] = distance
            
    def a_star(self, start, goal):
        self.reset_nodes()
            
        open_set = []
        closed_set = set()

        start.g_score = 0
        start.h_score = self.calculate_heuristic(start, goal)
        heapq.heappush(open_set, (start.g_score + start.h_score, start))

        while open_set:
            _, current_node = heapq.heappop(open_set)

            if current_node == goal:
                return self.reconstruct_path(start, goal)

            closed_set.add(current_node)

            for neighbor in self.get_neighbors(current_node):
                if neighbor in closed_set:
                    continue

                tentative_g_score = current_node.g_score + self.get_distance(current_node, neighbor)
                
                if tentative_g_score < neighbor.g_score:
                    neighbor.parent = current_node
                    neighbor.g_score = tentative_g_score
                    neighbor.h_score = self.calculate_heuristic(neighbor, goal)
                    f_score = neighbor.g_score + neighbor.h_score
                    heapq.heappush(open_set, (f_score, neighbor))

        return None
    
    def a_star_unknown(self, start):
        self.reset_nodes()
            
        open_set = []
        closed_set = set()

        start.g_score = 0
        start.h_score = self.calculate_heuristic(start, start)
        heapq.heappush(open_set, (start.g_score + start.h_score, start))

        while open_set:
            _, current_node = heapq.heappop(open_set)

            if current_node in self.open_nodes:
                return self.reconstruct_path(start, current_node)

            closed_set.add(current_node)

            for neighbor in self.get_neighbors(current_node):
                if neighbor in closed_set:
                    continue
                
                for node in self.nodes:
                    if node == neighbor:
                        neighbor = node

                tentative_g_score = current_node.g_score + self.get_distance(current_node, neighbor)

                if tentative_g_score < neighbor.g_score:
                    neighbor.parent = current_node
                    neighbor.g_score = tentative_g_score
                    neighbor.h_score = self.calculate_heuristic(neighbor, start)
                    f_score = neighbor.g_score + neighbor.h_score
                    heapq.heappush(open_set, (f_score, neighbor))
        
        return None
    
    def astar_beacon(self):
        beacons = sorted(list(self.beacon_nodes), key=lambda x: x[0])
        
        path = []
        for i in range(len(beacons)):
            start = beacons[i][1]
            goal = beacons[0][1] if i == len(beacons) - 1 else beacons[i + 1][1]
                
            result = self.a_star(start, goal)
            path.extend(result[:-1])
        path.append(beacons[0][1])
                
        return path     
    
    def reset_nodes(self):
        for node in self.nodes:
            node.g_score = float('inf')
            node.h_score = 0
            node.parent = None
            for node in self.get_neighbors(node):
                node.g_score = float('inf')
                node.h_score = 0
                node.parent = None  
    
    def calculate_heuristic(self, node, goal, method='euclidean'):
        if method == 'euclidean':
            return ((node.coordinates[0] - goal.coordinates[0]) ** 2 + (node.coordinates[1] - goal.coordinates[1]) ** 2) ** 0.5
        if method == 'manhattan':
            return abs(node.coordinates[0] - goal.coordinates[0]) + abs(node.coordinates[1] - goal.coordinates[1])
    
    def reconstruct_path(self, start, goal):
        path = []
        current_node = goal
        while current_node:
            path.insert(0, current_node)
            current_node = current_node.parent
        return path

    def get_neighbors(self, node):
        neighbors = set()
        for edge, _ in self.edges.items():
            if edge[0] == node:
                neighbors.add(edge[1])
        return neighbors

    def get_distance(self, node1, node2):
        return self.edges.get((node1, node2), float('inf'))
    
    def __str__(self):
        return "Nodes: " + str(self.nodes) + "\nEdges: " + str(self.edges)
    
    def __repr__(self):
        return str(self)

# Example usage:
if __name__ == '__main__':
    graph = Graph()

    # Create nodes and add them to the graph
    node1 = Node(0, 0)
    node2 = Node(2, 0)
    node3 = Node(4, 0)
    node4 = Node(4, 2)
    node5 = Node(4, 4)

    graph.add_node(node1)
    graph.add_node(node2)
    graph.add_node(node3)
    graph.add_node(node4)
    graph.add_node(node5)

    # Add edges between nodes with distances
    graph.add_edge(node1, node2, 2)
    graph.add_edge(node2, node3, 2)
    graph.add_edge(node3, node4, 2)
    graph.add_edge(node4, node5, 2)

    start_node = node1
    goal_node = node5

    path = graph.a_star(start_node, goal_node)

    if path:
        print("A* Path:", path)
    else:
        print("No path found.")


