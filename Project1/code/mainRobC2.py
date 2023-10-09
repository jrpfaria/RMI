
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

from utilsC2 import *

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

        map = [[" "] * MAP_COLS] * MAP_ROWS

        target = previous_target = (0, 0)
        offsets = (0 - self.measures.x, 0 - self.measures.y)

        map[MAP_ROWS // 2][MAP_COLS // 2] = "I"

        lineHistory = []        

        while True:
            self.readSensors()

            line = self.measures.lineSensor
            print_sensor_readings(line)

            # Register last 5 readingsfrom line sensor
            lineHistory.append(line)
            if len(lineHistory) > 5:
                lineHistory.pop(0)
            
            # Position data
            coordinates = (self.measures.x + offsets[0], self.measures.y + offsets[1]) 
            
            heading = self.measures.compass // 45

            if (coordinates == target):
                previous_target = target
                target = changeTarget(target, heading)

            # print(coordinates, heading)
            speed = calculateSpeed(coordinates, target, heading)
            error = calculateError(coordinates, target, previous_target)

            # print(speed, error)

            if '1' not in line[2:5]:
                turn = readjustToLine(lineHistory)
                self.driveMotors(turn[0], turn[1])
            else:
                self.driveMotors(speed - error, speed + error)


            # printMap(MAP)


            # self.finish()

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
