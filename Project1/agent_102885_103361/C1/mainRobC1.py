import math
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

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
        Kp = 3.0000  # Proportional constant
        Ki = 0.0000  # Integral constant
        Kd = 0.0000  # Derivative constant

        # Initialize variables for PID controller
        last_error = 0
        integral = 0

        step = 0.08
        n_sensor = 7
        base = get_base(n_sensor, step)
        base_speed = 0.15

        error_history = [0, 0, 0]
        max_history_size = 10

        error_history = [0, 0, 0]
        max_history_size = 10

        while True:

            # Read sensor values (0s and 1s)
            self.readSensors()

            # print(self.measures.lineSensor)

            line = self.measures.lineSensor
            # print_sensor_readings(line)

            # check if 1s star appearing from left to right
            # if so then we are going back

            if '1' not in line:
                angle = 0
                self.driveMotors(-0.03, -0.03)
                if any(error > 0.08 for error in error_history[-3:]):
                    self.readSensors()
                    self.driveMotors(-0.1, +0.1)
                if any(error < -0.08 for error in error_history[-3:]):
                    self.readSensors()
                    self.driveMotors(+0.1, -0.1)
                history = []  
                early = False            
                while angle > -3 * math.pi / 4 + 0.24:
                    self.readSensors()
                    line = self.measures.lineSensor
                    line = remove_outliers(line)
                    if '11' in ''.join(line):
                        error = center_of_mass(line, step, 0)
                        if len(history) > 0:
                            error = center_of_mass(line, step, 0)
                            if error > 0.24:
                                early = True
                                break
                        history.append(error)
                    self.driveMotors(-0.08, +0.08)
                    angle -= 0.16
                if early:
                    continue
                result, message = check_for_window_pattern(history)
                if message == 'Needs adjustment':
                    self.readSensors()
                    self.driveMotors(+0.15, -0.15)
                if result:
                    continue
                for _ in range(6):
                    self.readSensors()
                    self.driveMotors(+0.15, -0.15)
                    line = self.measures.lineSensor
                while line[-4:].count('1') < 2:
                    self.readSensors()
                    line = self.measures.lineSensor
                    self.driveMotors(+0.15, -0.15)
                    angle += 0.3
                continue

            # Calculate the error
            error = center_of_mass(line, step, base) - base

            error_history.append(error)

            if len(error_history) > max_history_size:
                error_history.pop(0)

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

            # Drive motors with adjusted power
            self.driveMotors(left_power, right_power)

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
