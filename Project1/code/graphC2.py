import heapq  # For priority queue

class Node:
    def __init__(self, x, y, state="unknown"):
        self.coordinates = (x, y)
        self.g_score = float('inf')  # Initialize to infinity
        self.h_score = 0  # Heuristic score
        self.parent = None
        self.state = state
        
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
        self.edges = {}  # Dictionary to store edges and their distances

    def add_node(self, node):
        self.nodes.add(node)

    def add_edge(self, node1, node2, distance = 2):
        self.edges[(node1, node2)] = distance
        self.edges[(node2, node1)] = distance  # Since the graph is undirected

    def update_node_state(self, node, new_state):
        if node in self.nodes:
            node.state = new_state
            
    def a_star(self, start, goal):
        open_set = []  # Priority queue to keep track of open nodes
        closed_set = set()  # Set to keep track of closed nodes

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

        return None  # If no path is found

    def a_star_with_state(self, start, target_state):
        open_set = []  # Priority queue to keep track of open nodes
        closed_set = set()  # Set to keep track of closed nodes

        start.g_score = 0
        start.h_score = self.calculate_heuristic(start, start)  # Heuristic from start to itself is 0
        heapq.heappush(open_set, (start.g_score + start.h_score, start))

        while open_set:
            _, current_node = heapq.heappop(open_set)

            if current_node.state == target_state:
                # Reconstruct and return the path to the closest node with the target state
                return self.reconstruct_path(start, current_node)

            closed_set.add(current_node)

            for neighbor in self.get_neighbors(current_node):
                if neighbor in closed_set:
                    continue

                tentative_g_score = current_node.g_score + self.get_distance(current_node, neighbor)

                if tentative_g_score < neighbor.g_score:
                    neighbor.parent = current_node
                    neighbor.g_score = tentative_g_score
                    neighbor.h_score = self.calculate_heuristic(neighbor, start)  # Using heuristic to start
                    f_score = neighbor.g_score + neighbor.h_score
                    heapq.heappush(open_set, (f_score, neighbor))

        return None
    
    def calculate_heuristic(self, node, goal, method='euclidean'):
        # You can use any heuristic function here; for simplicity, we use Manhattan distance.
        if method == 'euclidean':
            return ((node.coordinates[0] - goal.coordinates[0]) ** 2 + (node.coordinates[1] - goal.coordinates[1]) ** 2) ** 0.5
        if method == 'manhattan':
            return abs(node.coordinates[0] - goal.coordinates[0]) + abs(node.coordinates[1] - goal.coordinates[1])
    
    def get_nodes_with_state(self, state):
        return [node for node in self.nodes if node.state == state]
    
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


