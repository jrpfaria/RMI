import sys
from collections import deque
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
        paths = []                              # Paths
        prev_target = (0, 0)                    # Previous target
        target = (2, 0)                         # Target
        HISTORY_SIZE = 9                        # History size
        history = deque(maxlen=HISTORY_SIZE)    # History of sensor values
        
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

        g.add_node(prev_target)
        g.set_visited(prev_target)

        self.readSensors()
        if (self.measures.ground != -1):
            pmap[MAP_START_Y][MAP_START_X] = str(self.measures.ground)
            g.add_beacon(self.measures.ground, MAP_START)

        while True:
            self.readSensors()
            line = self.measures.lineSensor
            compass = self.measures.compass
            
            pmap, paths = update_map_start(line, compass, pmap, MAP_START, paths)
            if  -20 < compass < -4: break

            if abs(compass) % 45 < 15:
                self.driveMotors(-0.5, 0.5)
            else:
                self.driveMotors(-0.15, 0.15)
    
        if paths:
            g.add_connections(prev_target, paths)

            lx, ly = paths[0]
            target = (lx, ly)
        
        else:
            print("no paths?")

        while True:
            self.readSensors()

            # Get the line and compass values
            line = self.measures.lineSensor
            compass = radians(self.measures.compass) # Converted to radians to normalize calculations

            # Calculate the control constants
            if is_close(coordinates, target, 0.438):
                cm_Kp = 0
                ad_Kp = 0
                base_speed = 0.1
                n_sensors = 3
            elif is_close(coordinates, target, 1.6):
                cm_Kp = 2
                ad_Kp = 0
                base_speed = 0.10
                n_sensors = 3
            elif is_close(coordinates, target, 1.9):
                cm_Kp = 4
                ad_Kp = 0
                base_speed = 0.15
                n_sensors = 7
            else:
                cm_Kp = 5
                ad_Kp = 0
                base_speed = 0.15
                n_sensors = 3

            # Calculate the error
            evaluated_line = centered_line(line, n_sensors)
            cm_error = cm_Kp * center_of_mass(evaluated_line, STEP)
            ad_error = ad_Kp * angular_deviation(coordinates, target, theta if theta % (pi / 4) == 0 else compass)

            # Calculate the control output (PID)
            control = cm_error + ad_error

            # Calculate the sensor positions
            sensor_positions = calculate_sensor_positions(coordinates, theta, compass)

            # Add the sensor values to the history
            history.append((line, sensor_positions))

            # Calculate motor powers
            left_power, right_power = base_speed + control, base_speed - control

            # Ensure motor powers are within the valid range (-0.15 to 0.15)
            left_power = min(max(left_power, -0.15), +0.15)
            right_power = min(max(right_power, -0.15), +0.15)

            ### MOVEMENT MODEL ###
            coordinates, theta = general_movement_model(left_power, right_power, out, theta, coordinates)
            
            ### MAPPING CHALLENGE ### 
            if is_close(coordinates, target, 0.2):
                print(f"x: {coordinates[0]:.2f}, y: {coordinates[1]:.2f}")
                print(f"theta: {degrees(theta):.2f}")
                print(f"target: {target}")
                print(f"prev_target: {prev_target}")
                
                # Keep track of the previous pointers
                old_ptarget = prev_target
                old_target = target

                # Set the current position as visited
                g.set_visited(old_target)

                # Fixate coordinates with respect to axis of movement
                theta = fixate_theta(compass)
                coordinates = fixate_coordinates(coordinates, theta)

                # If target is a beacon, mark it as a beacon
                if (self.measures.ground != -1):
                    old_target_x, old_target_y = old_target
                    pmap[MAP_START_Y - old_target_y][MAP_START_X + old_target_x] = str(self.measures.ground)
                    g.add_beacon(self.measures.ground, old_target)
                
                # Scan the surrounding area for paths
                paths = scan_paths(list(history), old_target)
                paths = sort_paths(paths)

                print(f"paths: {paths}")

                translated_paths = translate_paths(paths, old_ptarget, old_target)
          
                # Candidate targets are the paths found that have not been visited
                candidate_targets = [path for path in translated_paths if path not in g.visited]

                # Set the previous target as the current target
                prev_target = old_target

                # If there are no candidate targets, add path to closest unknown node to targets buffer
                if not candidate_targets:
                    path, _ = g.bfs_unknowns(old_target)

                    if path is None:
                        # write path to beacons to file
                        # write map to file
                        self.finish()
                        break

                    path = path[1:]
                    print(f"bfs path: {path}")
                    target = path.pop(0)

                    while abs(error := angular_deviation(coordinates, target, theta)) > pi / 15:
                        self.readSensors()
                        error = max(min(error, 0.15), -0.15)
                        left_power, right_power = -error, error

                        coordinates, theta = general_movement_model(left_power, right_power, out, theta, coordinates)
                        out = (left_power, right_power)
                        self.driveMotors(*out)
                    
                    while path:
                        self.readSensors()
                        line = self.measures.lineSensor

                        if is_close(coordinates, target, 0.438):
                            cm_Kp = 0
                            ad_Kp = 0
                            base_speed = 0.1
                            n_sensors = 3
                        elif is_close(coordinates, target, 1.6):
                            cm_Kp = 2
                            ad_Kp = 0
                            base_speed = 0.10
                            n_sensors = 3
                        elif is_close(coordinates, target, 1.9):
                            cm_Kp = 4
                            ad_Kp = 0
                            base_speed = 0.15
                            n_sensors = 7
                        else:
                            cm_Kp = 5
                            ad_Kp = 0
                            base_speed = 0.15
                            n_sensors = 3

                        # Calculate the error
                        evaluated_line = centered_line(line, n_sensors)
                        cm_error = cm_Kp * center_of_mass(evaluated_line, STEP)
                        ad_error = ad_Kp * angular_deviation(coordinates, target, theta if theta % (pi / 4) == 0 else compass)

                        print(f"theta: {degrees(theta):.2f}")

                        # Calculate the control output (PID)
                        control = cm_error + ad_error

                        # Calculate motor powers
                        left_power, right_power = base_speed + control, base_speed - control

                        # Ensure motor powers are within the valid range (-0.15 to 0.15)
                        left_power = min(max(left_power, -0.15), +0.15)
                        right_power = min(max(right_power, -0.15), +0.15)

                        ### MOVEMENT MODEL ###
                        coordinates, theta = general_movement_model(left_power, right_power, out, theta, coordinates)

                        if is_close(coordinates, target, 0.2):
                            prev_target = target
                            target = path.pop(0) 

                            while abs(error := angular_deviation(coordinates, target, theta)) > pi / 15:
                                self.readSensors()
                                error = max(min(error, 0.15), -0.15)
                                left_power, right_power = -error, error

                                coordinates, theta = general_movement_model(left_power, right_power, out, theta, coordinates)
                                out = (left_power, right_power)
                                self.driveMotors(*out)

                        out = (left_power, right_power)
                        self.driveMotors(*out)    
                    continue
                        
                target = candidate_targets.pop(0)

                # if is_far(coordinates, target, 3):
                #     target = help_robot(coordinates, compass)

                while abs(error := angular_deviation(coordinates, target, theta)) > pi / 15:
                    self.readSensors()
                    error = max(min(error, 0.15), -0.15)
                    left_power, right_power = -error, error

                    coordinates, theta = general_movement_model(left_power, right_power, out, theta, coordinates)
                    out = (left_power, right_power)
                    self.driveMotors(*out)
                    
                g.add_connections(old_target, candidate_targets + [target])

                print(f"target: {target}")

                pmap = update_map(paths, pmap, MAP_START, old_ptarget, old_target)
                write_map_to_file(pmap, FILENAME)
                # go forward a bit


            # Fixate coordinates with respect to axis of movement
            if (cm_error == 0 and ad_error == 0) or "1" == line[0] or "1" == line[6]:
                theta = fixate_theta(compass)
                coordinates = fixate_coordinates(coordinates, theta)

            ### END MAPPING CHALLENGE ###
            
            out = (left_power, right_power) # used to help with out(t-1)
            ## END MOVEMENT MODEL

            # Drive motors with adjusted power
            self.driveMotors(*out)

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
