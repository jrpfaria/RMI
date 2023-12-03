class Node:
    def __init__(self, x, y):
        self.coordinates = (x, y)
        self.g_score = float('inf')  # Initialize to infinity
        self.h_score = 0  # Heuristic score
        self.parent = None

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
        self.edges = {}

    def add_node(self, node):
        self.nodes.add(node)

    def add_edge(self, node1, node2, cost):
        self.edges.setdefault(node1, []).append((node2, cost))
        self.edges.setdefault(node2, []).append((node1, cost))
