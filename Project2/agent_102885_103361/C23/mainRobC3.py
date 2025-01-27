
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

from utils import *
from graph import *

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        self.readSensors()

        MAP_ROWS=21
        MAP_COLS=49

        c2_map = [[" " for _ in range(MAP_COLS)] for _ in range(MAP_ROWS)]

        graph = Graph()     
        graph.set_node_beacon_count(self.nBeacons)

        prev_target = aux = Node(0, 0)
        graph.add_node(aux)
        graph.set_node_visited(aux)
        
        if (self.measures.ground != -1):
            graph.set_node_beacon(aux, self.measures.ground)
        
        target = None
        
        offsets = (self.measures.x, self.measures.y)

        map_start_x = 24
        map_start_y = 10

        map_start = (map_start_x, map_start_y)
        
        c2_map[map_start_y][map_start_x] = "I"

        lineHistory = []   
        
        paths = []

        while True:
            self.readSensors()
            line = self.measures.lineSensor
            compass = self.measures.compass
            
            c2_map, paths = addToMapStart(line, compass, c2_map, map_start, paths)
            if  -20 < compass < 0: break

            if abs(compass) % 45 < 15:
                self.driveMotors(-0.05, 0.05)
            else:
                self.driveMotors(-0.15, 0.15)
    
        if paths:
            for x, y in paths:
                node = Node(x, y)
                graph.add_node(node)
                graph.add_edge(aux, node)

            lx, ly = paths[0]
            target = Node(lx, ly)
        else:
            print("no maidens?")
            
        while True:
            self.readSensors()

            line = self.measures.lineSensor

            current = Node(self.measures.x - offsets[0], self.measures.y - offsets[1]) 

            sensor_positions = calculate_sensor_positions(current, self.measures.compass)
            lineHistory.append((line, sensor_positions))

            if len(lineHistory) > 7:
                lineHistory.pop(0)

            prev_target = aux
            dist_to_target = euclidean_distance(current, target)

            in_speed_zone = dist_to_target > 1
            in_vicinity = dist_to_target < 0.2
            
            error = calculateError(current, target, self.measures.compass)

            while (in_vicinity):
                paths = evaluateLineHistory(lineHistory, target)
                aux = target

                unknowns = pickPath(paths, prev_target, target)
                c2_map = addToMap(paths, c2_map, map_start, prev_target, target)
                cost = 3
                
                if target in graph.open_nodes:
                    graph.set_node_visited(target)
                
                if (self.measures.ground != -1):
                    graph.set_node_beacon(target, self.measures.ground)

                target = prev_target
                
                for x, y, score in unknowns:
                    new_node = Node(x, y)
                    graph.add_node(new_node)
                    graph.add_edge(aux, new_node)
                    
                    # decide which edge to take
                    if (score <= cost and new_node not in graph.closed_nodes):
                        cost = score
                        target = new_node
                        
                if target == prev_target:
                    path = graph.a_star_unknown(aux)
                    # a star gives you the shortest path to the node
                    
                    if path is None:
                        
                        path = graph.a_star(aux, Node(0, 0))

                        target = path.pop(0)
                        while path:
                            self.readSensors()
                            current = Node(self.measures.x - offsets[0], self.measures.y - offsets[1])
                            in_vicinity = euclidean_distance(current, target) < 0.2
                            if in_vicinity:
                                aux = target
                                target = path.pop(0)
                            error = calculateError(current, target, self.measures.compass)
                            speed = 0.15
                            self.driveMotors(speed - error, speed + error)
                            

                        while not euclidean_distance(current, target) < 0.2:
                            self.readSensors()
                            current = Node(self.measures.x - offsets[0], self.measures.y - offsets[1])
                            error = calculateError(current, target, self.measures.compass)
                            speed = 0.15
                            self.driveMotors(speed - error, speed + error)


                        path = graph.astar_beacon()
                        write_beacon_path_to_file(path)
                        self.finish()
                        quit()
                        
                    target = path.pop(0)
                    while path:
                        self.readSensors()
                        current = Node(self.measures.x - offsets[0], self.measures.y - offsets[1])
                        in_vicinity = euclidean_distance(current, target) < 0.2
                        if in_vicinity:
                            aux = target
                            target = path.pop(0)
                        error = calculateError(current, target, self.measures.compass)
                        speed = 0.15
                        self.driveMotors(speed - error, speed + error)

                
                print_map(c2_map)
                break
            
            speed = 0.15 if in_speed_zone else 0.1
            self.driveMotors(speed - error, speed + error)

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
