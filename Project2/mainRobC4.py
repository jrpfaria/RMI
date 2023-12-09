import math
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from graph import *
from utils import *

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

        # Constants for PID controller
        STEP = 0.08

        # Read sensor values (0s and 1s)
        self.readSensors()

        while not self.measures.start:
            # Read sensor values (0s and 1s)
            self.readSensors()

        # Name of the file to write the map to
        FILENAME = "map.out"

        # Initialize variables for movement model
        coordinates = (0, 0)    # Coordinates
        out = (0, 0)            # Step
        theta = 0               # Angle

        # Initialize variables for mapping
        freely_moving = False   # Flag to indicate if robot is just moving forward
        paths = []              # Paths
        prev_target = (0,0)     # Previous target
        target = (2,0)          # Target
        history = []            # History of directions
        
        # Initialize variables for robot movement
        left_power = 0          # Left motor power
        right_power = 0         # Right motor power

        MAP_ROWS=21
        MAP_COLS=49
        MAP_SHAPE=(MAP_ROWS, MAP_COLS)
        MAP_START_X=24
        MAP_START_Y=10
        MAP_START = (MAP_START_X, MAP_START_Y)

        pmap = [[" " for _ in range(MAP_COLS)] for _ in range(MAP_ROWS)]

        pmap[MAP_START_Y][MAP_START_X] = "I"

        g = Graph()

        if (self.measures.ground != -1):
            pmap[MAP_START_Y][MAP_START_X] = str(self.measures.ground)
            g.add_beacon(self.measures.ground, MAP_START)

        while True:
            self.readSensors()

            # Get the line and compass values
            line = self.measures.lineSensor
            compass = radians(self.measures.compass) # Converted to radians to normalize calculations

            history.append(line)
            if len(history) > 7:
                history.pop(0)

            # Calculate the error
            if is_close(coordinates, target, 0.438):
                Kp = 0
                base_speed = 0.06
                n_sensors = 3
            elif is_close(coordinates, target, 1.6):
                Kp = 2
                base_speed = 0.1
                n_sensors = 3
            elif is_close(coordinates, target, 1.9):
                Kp = 4
                base_speed = 0.15
                n_sensors = 7
            else:
                Kp = 5
                base_speed = 0.15
                n_sensors = 3

            evaluated_line = centered_line(line, n_sensors)
            error = center_of_mass(evaluated_line, STEP)

            # Calculate the control output (PID)
            control = Kp * error

            # Calculate motor powers
            left_power = base_speed + control
            right_power = base_speed - control

            # Ensure motor powers are within the valid range (-0.15 to 0.15)
            left_power = min(max(left_power, -0.15), +0.15)
            right_power = min(max(right_power, -0.15), +0.15)

            ### MOVEMENT MODEL ###
            coordinates, theta = general_movement_model(left_power, right_power, out, theta, coordinates)

            print(f"target: {target}")
            print(f"prev_target: {prev_target}")
            
            ### MAPPING CHALLENGE ### 
            if is_close(coordinates, target, 0.2):
                
                if (self.measures.ground != -1):
                    target_x, target_y = target
                    pmap[target_y][target_x] = str(self.measures.ground)
                    g.add_beacon(self.measures.ground, target)

                theta = fixate_theta(compass)
                coordinates = fixate_coordinates(coordinates, theta)
                
                paths = find_paths(history)
                print(f"paths: {paths}")
                
                unknowns = get_paths(paths, prev_target, target)
                pmap = update_map(paths, pmap, MAP_START, prev_target, target)

                write_map_to_file(pmap, FILENAME)

                g.add_nodes_from(unknowns)
                ## g.add_edges_from
                # check best unknown path    

                prev_target = target
                
                target = next_target(unknowns)

                if target is None or target in g.visited:
                    target = prev_target
                
                # target = next_target(target, paths)
                # atan2 : on_spot_error(coordinates, target, theta)

                error = degrees(on_spot_error(coordinates, target, theta))

                while (abs(error) > 15):
                    # print(f"error: {error}")
                    self.readSensors()
                    error = max(min(error, 0.15), -0.15)
                    left_power, right_power = (-error, error)
                    coordinates, theta = general_movement_model(left_power, right_power, out, theta, coordinates)
                    out = (left_power, right_power)
                    self.driveMotors(left_power, right_power)
                    error = degrees(on_spot_error(coordinates, target, theta))
                
                # go forward a bit


            # Fixate coordinates with respect to axis of movement
            if error == 0 or "1" == line[0] or "1" == line[6]:
                theta = fixate_theta(compass)
                coordinates = fixate_coordinates(coordinates, theta)

            ### END MAPPING CHALLENGE ###
            
            out = (left_power, right_power) # used to help with out(t-1)
            ## END MOVEMENT MODEL

            # Drive motors with adjusted power
            self.driveMotors(left_power, right_power)

            # print(f"x: {coordinates[0]:.2f}, y: {coordinates[1]:.2f}")
            # print(f"theta: {degrees(theta):.2f}")

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
