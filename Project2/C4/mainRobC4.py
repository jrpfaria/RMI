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

        # Read sensor values (0s and 1s)
        self.readSensors()

        while not self.measures.start:
            # Read sensor values (0s and 1s)
            self.readSensors()

        # Name of the file to write the map to
        MAPPING_FILENAME = "map.out"
        PLANNING_FILENAME = "plan.out"

        # Initialize variables for mapping
        paths = []                              # Paths
        prev_target = (0, 0)                    # Previous target
        target = (2, 0)                         # Target
        line = self.measures.lineSensor         # Line sensor values
        HISTORY_SIZE = 9                        # History size
        history = deque(maxlen=HISTORY_SIZE)    # History of sensor values
        candidate_targets = []                  # Candidate targets
        map_updates = {}                        # Last updates
        translated_map_updates = {}             # Translated updates
        
        # Initialize variables for robot movement
        coordinates = (0, 0)                # Coordinates
        out = (0, 0)                        # Step
        theta = 0                           # Angle
        compass = 0                         # Compass
        left_power = 0                      # Left motor power
        right_power = 0                     # Right motor power
        ROTATION_SPEED = 0.001              # Rotation speed
        MAX_SPEED = 0.15                    # Max speed
        ROTATION_THRESHOLD = pi / 12        # Rotation threshold
        ROTATION_SLOWDOWN_THRESHOLD = 20    # Rotation slowdown threshold
        STEP = 0.08

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
        g.set_start(prev_target)

        g.set_beacon_count(self.nBeacons)

        def rotate_on_spot():
            nonlocal coordinates, target, compass, ROTATION_SPEED, ROTATION_SLOWDOWN_THRESHOLD, MAX_SPEED, out, theta, left_power, right_power, line
            while abs(error := angular_deviation(coordinates, target, compass)) > ROTATION_THRESHOLD:
                self.readSensors()
                line = self.measures.lineSensor
                compass = radians(self.measures.compass)

                error = calculate_rotation_error(error, degrees(compass), ROTATION_SPEED, ROTATION_SLOWDOWN_THRESHOLD, MAX_SPEED)

                left_power, right_power = -error, error

                coordinates, theta = general_movement_model(left_power, right_power, out, theta, coordinates)
                out = (left_power, right_power)
                self.driveMotors(*out)
        
        def move_to_closest_unknown():
            nonlocal coordinates, target, compass, ROTATION_SPEED, ROTATION_SLOWDOWN_THRESHOLD, MAX_SPEED, out, theta, left_power, right_power, prev_target, target, STEP, g, line
            # Get the path to the closest unknown
            path, _ = g.bfs_unknowns(prev_target)
            path = path[1:]

            # If there is no path to an unknown, get the path to the start
            if path is None:
                move_to_start_and_finish()

            # Set the target to the next node in the path
            target = path.pop(0)

            # Rotate to face the target
            rotate_on_spot()

            # Move to the target
            while path:
                self.readSensors()
                compass = radians(self.measures.compass)
                line = self.measures.lineSensor
                
                cm_Kp, ad_Kp, base_speed, n_sensors = calculate_control_constants(coordinates, target, MAX_SPEED)

                # Calculate the control output (PID)
                control, cm_error, ad_error = calculate_control(line, n_sensors, coordinates, target, compass, cm_Kp, ad_Kp, STEP)

                # Calculate motor powers
                left_power, right_power = base_speed + control, base_speed - control

                # Ensure motor powers are within the valid range (-MAX_SPEED to MAX_SPEED)
                left_power, right_power = cap_speed(left_power, right_power, MAX_SPEED)

                # Update the coordinates and angle
                coordinates, theta = general_movement_model(left_power, right_power, out, theta, coordinates)

                if is_close(coordinates, target, 0.2):
                    prev_target = target
                    target = path.pop(0) 

                    rotate_on_spot()

                if cm_error == 0:
                    theta = fixate_theta(compass)
                    coordinates = fixate_coordinates(coordinates, theta)

                out = (left_power, right_power)
                self.driveMotors(*out)

        def move_to_start_and_finish():
            nonlocal coordinates, target, compass, ROTATION_SPEED, ROTATION_SLOWDOWN_THRESHOLD, MAX_SPEED, out, theta, left_power, right_power, prev_target, target, PLANNING_FILENAME, STEP, g, line
            # Get the path to the start
            path = g.astar(old_target, g.start)

            ### BEACON MOVEMENT ###
            beacon_path, _ = g.astar_beacons()
            write_beacon_path_to_file(beacon_path, PLANNING_FILENAME)
                        
            if path is None:
                self.finish()
                quit()

            path = path[1:]
            print(f"beacon path: {path}")
            target = path.pop(0)

            # Rotate to face the target
            rotate_on_spot()

            # Move to the target
            while path:
                self.readSensors()
                compass = radians(self.measures.compass)
                line = self.measures.lineSensor
                
                cm_Kp, ad_Kp, base_speed, n_sensors = calculate_control_constants(coordinates, target, MAX_SPEED)

                # Calculate the control output (PID)
                control, cm_error, ad_error = calculate_control(line, n_sensors, coordinates, target, compass, cm_Kp, ad_Kp, STEP)

                # Calculate motor powers
                left_power, right_power = base_speed + control, base_speed - control

                # Ensure motor powers are within the valid range (-MAX_SPEED to MAX_SPEED)
                left_power, right_power = cap_speed(left_power, right_power, MAX_SPEED)

                # Update the coordinates and angle
                coordinates, theta = general_movement_model(left_power, right_power, out, theta, coordinates)

                if is_close(coordinates, target, 0.2):
                    prev_target = target
                    target = path.pop(0) 

                    rotate_on_spot()

                if cm_error == 0:
                    theta = fixate_theta(compass)
                    coordinates = fixate_coordinates(coordinates, theta)

                out = (left_power, right_power)
                self.driveMotors(*out)

            # write map to file
            self.finish()        
            quit()

        ### START CHALLENGE ##
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

            error = ROTATION_SPEED if compass % 45 < ROTATION_SLOWDOWN_THRESHOLD else MAX_SPEED

            left_power, right_power = -error, error

            coordinates, theta = general_movement_model(left_power, right_power, out, theta, coordinates)
            out = (left_power, right_power)
            self.driveMotors(*out)

        if paths:
            g.add_connections(prev_target, paths)
            target = paths[0]
        
        else:
            print("no paths?")

        while True:
            self.readSensors()

            # Get the line and compass values
            line = self.measures.lineSensor
            compass = radians(self.measures.compass) # Converted to radians to normalize calculations

            # Calculate the sensor positions
            sensor_positions = calculate_sensor_positions(coordinates, theta, compass)

            # Add the sensor values to the history
            history.append((line, sensor_positions))

            # Calculate the control constants
            cm_Kp, ad_Kp, base_speed, n_sensors = calculate_control_constants(coordinates, target, MAX_SPEED)

            # Calculate the control output (PID)
            control, cm_error, ad_error = calculate_control(line, n_sensors, coordinates, target, compass, cm_Kp, ad_Kp, STEP)

            # Calculate motor powers
            left_power, right_power = base_speed + control, base_speed - control

            # Ensure motor powers are within the valid range (-MAX_SPEED to MAX_SPEED)
            left_power, right_power = cap_speed(left_power, right_power, MAX_SPEED)

            # Update the coordinates and angle
            coordinates, theta = general_movement_model(left_power, right_power, out, theta, coordinates)
            
            print(f"NML | x: {coordinates[0]:.2f}, y: {coordinates[1]:.2f}")

            ### NORMAL MOVEMENT ### 
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

                translated_paths, translated_map_updates = translate_paths(paths, old_ptarget, old_target)
          
                # Candidate targets are the paths found that have not been visited
                candidate_targets = [path for path in translated_paths if path not in g.visited]

                # Set the previous target as the current target
                prev_target = old_target

                # If there are no candidate targets, add path to closest unknown node to targets buffer
                if not candidate_targets:
                    move_to_closest_unknown()
                    continue
                        
                target = candidate_targets.pop(0)

                rotate_on_spot()
                
                g.add_connections(old_target, candidate_targets + [target])

                print(f"target: {target}")

                pmap, map_updates = update_map(paths, pmap, MAP_START, old_ptarget, old_target)
                write_map_to_file(pmap, MAPPING_FILENAME)
                # go forward a bit

                self.readSensors()
                line = self.measures.lineSensor
                left_power, right_power = 0.15, 0.15
                coordinates, theta = general_movement_model(left_power, right_power, out, theta, coordinates)
                out = (left_power, right_power)
                self.driveMotors(*out)

                ### Filter                     
                print("TIAGOVSKI PAYSAFES")
                print(f"{target=}")
                while True:
                    if "1" in line[2:5]:
                        print(line)
                        target = correct_target(prev_target, compass)
                        print(f"{target=}")
                        break

                    g.remove_edge(prev_target, target)
                    print(f"{paths=}")
                    print(f"{candidate_targets=}")
                    print(f"{target=}")
                    print(f"{translated_map_updates=}")
                    print(f"{map_updates=}")
                    dt = translated_map_updates[target]
                    dt_pos = map_updates[dt]
                    dt_y, dt_x = dt_pos
                    pmap[dt_y][dt_x] = " "

                    self.readSensors()
                    line = self.measures.lineSensor
                    compass = self.measures.compass


                    if not candidate_targets:
                        while True:
                            print("copas")
                            self.readSensors()
                            line = self.measures.lineSensor
                            compass = self.measures.compass
                            
                            pmap, paths = update_map_start(line, compass, pmap, MAP_START, paths)

                            if  -20 < compass < -4: break

                            error = ROTATION_SPEED if compass % 45 < ROTATION_SLOWDOWN_THRESHOLD else MAX_SPEED

                            left_power, right_power = -error, error

                            coordinates, theta = general_movement_model(left_power, right_power, out, theta, coordinates)
                            out = (left_power, right_power)
                            self.driveMotors(*out)
                        break
                    
                    target = candidate_targets.pop(0)

                    while abs(error := angular_deviation(coordinates, target, compass)) > ROTATION_THRESHOLD:
                        self.readSensors()
                        line = self.measures.lineSensor
                        compass = radians(self.measures.compass)

                        error = calculate_rotation_error(error, degrees(compass), ROTATION_SPEED, ROTATION_SLOWDOWN_THRESHOLD, MAX_SPEED)

                        left_power, right_power = -error, error

                        coordinates, theta = general_movement_model(left_power, right_power, out, theta, coordinates)
                        out = (left_power, right_power)
                        self.driveMotors(*out)

            # Fixate coordinates with respect to axis of movement
            if cm_error == 0:
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
