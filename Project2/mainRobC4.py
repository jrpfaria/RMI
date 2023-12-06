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
        Kp = 3.0000 # Proportional constant
        Ki = 0.0000  # Integral constant
        Kd = 0.0000  # Derivative constant

        # Initialize variables for PID controller
        last_error = 0
        integral = 0

        step = 0.08
        n_sensor = 3 # 7
        base = get_base(n_sensor, step)
        base_speed = 0.1

        # Read sensor values (0s and 1s)
        self.readSensors()

        while not self.measures.start:
            # Read sensor values (0s and 1s)
            self.readSensors()


        # Initialize variables for movement model
        wpow = (0, 0)           # Wheel Power
        coordinates = (0, 0)    # Coordinates
        out = (0, 0)            # Step
        theta = 0               # Angle

        # Initialize variables for mapping
        paths = []              # Paths
        prev_target = (0,0)     # Previous target
        target = (2,0)          # Target
        history = []            # History of directions
        
        MAP_ROWS=21
        MAP_COLS=49
        MAP_SHAPE=(MAP_ROWS, MAP_COLS)
        MAP_START_X=24
        MAP_START_Y=10
        MAP_START = (MAP_START_X, MAP_START_Y)

        pmap = [[" " for _ in range(MAP_COLS)] for _ in range(MAP_ROWS)]

        pmap[MAP_START_Y][MAP_START_X] = "I"

        g = Graph()

        while True:
            self.readSensors()

            # Get the line and compass values
            line = self.measures.lineSensor
            compass = radians(self.measures.compass) # Converted to radians to normalize calculations

            history.append(line)
            if len(history) > 7:
                history.pop(0)
            
            # Calculate the error
            error = center_of_mass(line[2:5], step, base) - base

            # Calculate the integral term
            integral += error

            # Calculate the derivative term
            derivative = error - last_error
            last_error = error

            # Calculate the control output (PID)
            control = Kp * error + Ki * integral + Kd * derivative

            # Calculate motor powers
            left_power = base_speed + control
            right_power = base_speed - control

            # Ensure motor powers are within the valid range (-0.15 to 0.15)
            left_power = min(max(left_power, -0.15), +0.15)
            right_power = min(max(right_power, -0.15), +0.15)

            # MOVEMENT MODEL
            # Wheel Power for current cycle
            wpow = (left_power, right_power)

            # Account for inertia
            out = movement_model(out, wpow)

            # Calculate the orientation after current cycle
            theta += rotation_model(out)
            theta = adjust_angle(theta)

            # Estimate coordinates after current cycle
            coordinates = gps_model(coordinates, out, theta)

            print(f"target: {target}")
            print(f"prev_target: {prev_target}")
            
            ### MAPPING CHALLENGE ### 
            if is_close(coordinates, target, 0.2):
                paths = find_paths(history)
                
                unknowns = get_paths(paths, prev_target, target)
                pmap = update_map(paths, pmap, MAP_START, prev_target, target)
                g.add_nodes_from(unknowns)
                ## g.add_edges_from
                # check best unknown path    

                prev_target = target
                
                target = next_target(unknowns)

                if target is None or target in g.visited:
                    target = prev_target
                
                # target = next_target(target, paths)
                # atan2 : on_spot_error(coordinates, target, theta)

                if target == prev_target:
                    pass
                    # rotate 180 degrees
                
                while ((error := degrees(on_spot_error(coordinates, target, compass))) > 15 or error < -15):
                    self.readSensors()
                    compass = radians(self.measures.compass)
                    error = max(min(error, 0.08), -0.08)
                    self.driveMotors(-error, error)



            # Fixate coordinates with respect to axis of movement
            if error == 0:
                theta = fixate_theta(compass) # :)
                coordinates = fixate_coordinates(coordinates, theta, prev_target)

            ### END MAPPING CHALLENGE ###
            
            out = (left_power, right_power) # used to help with out(t-1)
            ## END MOVEMENT MODEL

            # Drive motors with adjusted power
            self.driveMotors(left_power, right_power)

            print(f"x: {coordinates[0]:.2f}, y: {coordinates[1]:.2f}")
            print(f"theta: {degrees(theta):.2f}")
            print(paths)

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
